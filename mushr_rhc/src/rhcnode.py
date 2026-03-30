#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import cProfile
import os
from threading import Lock

import rclpy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA, Empty
from std_srvs.srv import Empty as SrvEmpty
from visualization_msgs.msg import Marker

import rhcbase
import rhctensor
import utils


class RHCNode(rhcbase.RHCBase):
    def __init__(self, dtype):
        super().__init__(dtype)

        self._declare_node_parameters()
        self._load_node_parameters()

        self.reset_lock = Lock()
        self.inferred_pose_lock = Lock()
        self._inferred_pose = None

        self.cur_rollout = None
        self.cur_rollout_ip = None
        self.traj_pub_lock = Lock()

        self.goal_active = False
        self.rhctrl = None
        self.T = None
        self.pr = None

        self.start_profile()
        self.setup_pub_sub()
        self.rhctrl = self.load_controller()
        self.T = self.get_int("T")
        self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)
        self.get_logger().info("Initialized")

    def _declare_node_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("profile", False),
                ("car_name", "car"),
                ("inferred_pose_t", "particle_filter_node/inferred_pose"),
                ("ctrl_topic", "input/navigation"),
                ("traj_chosen_topic", "~/traj_chosen"),
                ("control_rate_hz", 50.0),
            ],
        )

    def _load_node_parameters(self):
        self.do_profile = self.get_parameter("profile").value
        self.car_name = self.get_parameter("car_name").value
        self.inferred_pose_t = self.get_parameter("inferred_pose_t").value
        self.ctrl_topic = self.get_parameter("ctrl_topic").value
        self.traj_chosen_topic = self.get_parameter("traj_chosen_topic").value
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

    def start_profile(self):
        if self.do_profile:
            self.get_logger().warning("Running with profiling")
            self.pr = cProfile.Profile()
            self.pr.enable()

    def end_profile(self):
        if self.pr is not None:
            self.pr.disable()
            self.pr.dump_stats(os.path.expanduser("~/mushr_rhc_stats.prof"))

    def setup_pub_sub(self):
        self.create_service(SrvEmpty, "~/reset/soft", self.srv_reset_soft)
        self.create_service(SrvEmpty, "~/reset/hard", self.srv_reset_hard)

        self.create_subscription(PoseStamped, "/move_base_simple/goal", self.cb_goal, 1)
        self.create_subscription(
            PoseStamped,
            f"/{self.car_name}/{self.inferred_pose_t}",
            self.cb_pose,
            10,
        )

        self.rp_ctrls = self.create_publisher(
            AckermannDriveStamped,
            f"/{self.car_name}/{self.ctrl_topic}",
            2,
        )
        self.traj_chosen_pub = self.create_publisher(Marker, self.traj_chosen_topic, 10)
        self.expr_at_goal = self.create_publisher(Empty, "experiments/finished", 1)

    def control_loop(self):
        if self.rhctrl is None or not self.goal_active:
            return

        ip = self.inferred_pose()
        if ip is None:
            return

        with self.reset_lock:
            if not self.goal_active:
                return
            next_traj, rollout = self.rhctrl.step(ip)

        with self.traj_pub_lock:
            if rollout is not None:
                self.cur_rollout = rollout.clone()
                self.cur_rollout_ip = ip

        if next_traj is None:
            return

        self.publish_traj(next_traj, rollout)
        if self.rhctrl.at_goal(self.inferred_pose()):
            self.expr_at_goal.publish(Empty())
            self.publish_stop()
            self.goal_active = False

    def srv_reset_hard(self, request, response):
        self.get_logger().info("Start hard reset")
        with self.reset_lock:
            self.rhctrl = self.load_controller()
            self.goal_active = False
            self.publish_stop()
        self.get_logger().info("End hard reset")
        return response

    def srv_reset_soft(self, request, response):
        self.get_logger().info("Start soft reset")
        with self.reset_lock:
            if self.rhctrl is not None:
                self.rhctrl.reset()
            self.goal_active = False
            self.publish_stop()
        self.get_logger().info("End soft reset")
        return response

    def cb_goal(self, msg):
        goal = self.dtype(utils.rospose_to_posetup(msg.pose))
        if self.rhctrl is None:
            return
        if not self.rhctrl.set_goal(goal):
            self.get_logger().error("That goal is unreachable, please choose another")
            return
        self.get_logger().info("Goal set")
        self.goal_active = True

    def cb_pose(self, msg):
        self.set_inferred_pose(self.dtype(utils.rospose_to_posetup(msg.pose)))

        if self.cur_rollout is not None and self.cur_rollout_ip is not None:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            with self.traj_pub_lock:
                pts = (self.cur_rollout[:, :2] - self.cur_rollout_ip[:2]) + self.inferred_pose()[:2]

            marker.points = [Point(x=float(xy[0]), y=float(xy[1])) for xy in pts]
            r, g, b = 0x36, 0xCD, 0xC4
            marker.colors = [
                ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=0.7)
            ] * len(marker.points)
            marker.scale.x = 0.05
            self.traj_chosen_pub.publish(marker)

    def publish_traj(self, traj, rollout):
        assert traj.size() == (self.T, 2)
        assert rollout.size() == (self.T, 3)

        ctrl = traj[0]
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = self.get_clock().now().to_msg()
        ctrlmsg.drive.speed = float(ctrl[0])
        ctrlmsg.drive.steering_angle = float(ctrl[1])
        self.rp_ctrls.publish(ctrlmsg)

    def publish_stop(self):
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = self.get_clock().now().to_msg()
        ctrlmsg.drive.speed = 0.0
        ctrlmsg.drive.steering_angle = 0.0
        self.rp_ctrls.publish(ctrlmsg)

    def set_inferred_pose(self, ip):
        with self.inferred_pose_lock:
            self._inferred_pose = ip

    def inferred_pose(self):
        with self.inferred_pose_lock:
            return self._inferred_pose

    def destroy_node(self):
        self.end_profile()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RHCNode(rhctensor.float_tensor())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
