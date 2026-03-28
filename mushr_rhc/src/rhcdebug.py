#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import matplotlib.cm as cm
import matplotlib.colors as mplcolors
import rclpy
import torch
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

import rhcbase
import rhctensor
import utils


class RHCDebug(rhcbase.RHCBase):
    def __init__(self, dtype):
        super().__init__(dtype)

        self._declare_debug_parameters()
        self._load_debug_parameters()

        self.traj_chosen = None
        self.traj_chosen_id = 1
        self.inferred_pose = None
        self.init_pose = None
        self.goal = None

        self.current_path = Marker()
        self.current_path.header.frame_id = "map"
        self.current_path.type = self.current_path.LINE_STRIP
        self.current_path.action = self.current_path.ADD
        self.current_path.id = 1
        self.current_path.pose.position.x = 0.0
        self.current_path.pose.position.y = 0.0
        self.current_path.pose.position.z = 0.0
        self.current_path.pose.orientation.x = 0.0
        self.current_path.pose.orientation.y = 0.0
        self.current_path.pose.orientation.z = 0.0
        self.current_path.pose.orientation.w = 1.0
        self.current_path.color.a = 1.0
        self.current_path.color.r = 1.0
        self.current_path.scale.x = 0.03

        self.rhctrl = self.load_controller()

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.cb_initialpose, 10
        )
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self.cb_goal, 1)

        if self.debug_current_path:
            self.create_subscription(
                PoseStamped,
                self.inferred_pose_t,
                self.cb_inferred_pose,
                10,
            )
            self.current_path_pub = self.create_publisher(Marker, "~/current_path", 10)

        self.goal_pub = self.create_publisher(Marker, "~/goal", 10)

        if self.viz_traj_chosen_trace_enabled:
            self.create_subscription(
                PoseArray, self.traj_chosen_topic, self.cb_traj_chosen, 10
            )
            self.traj_chosen_pub = self.create_publisher(Marker, "~/traj_chosen", 10)
            self.traj_trace_timer = self.create_timer(
                1.0 / self.traj_chosen_trace_rate_hz, self.viz_traj_chosen_trace_cb
            )

        if self.viz_cost_fn_enabled:
            self.cost_fn_timer = self.create_timer(1.0 / 100.0, self.viz_cost_fn_cb)

    def _declare_debug_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("debug/flag/rollouts_on_init_pose", False),
                ("debug/flag/current_path", False),
                ("debug/flag/viz_traj_chosen_trace", True),
                ("debug/flag/viz_cost_fn", False),
                ("debug/traj_chosen_trace/rate", 10),
                ("inferred_pose_t", "particle_filter_node/inferred_pose"),
                ("traj_chosen_topic", "traj_chosen"),
            ],
        )

    def _load_debug_parameters(self):
        self.debug_rollouts = self.get_parameter("debug/flag/rollouts_on_init_pose").value
        self.debug_current_path = self.get_parameter("debug/flag/current_path").value
        self.viz_traj_chosen_trace_enabled = self.get_parameter(
            "debug/flag/viz_traj_chosen_trace"
        ).value
        self.viz_cost_fn_enabled = self.get_parameter("debug/flag/viz_cost_fn").value
        self.traj_chosen_trace_rate_hz = float(
            self.get_parameter("debug/traj_chosen_trace/rate").value
        )
        self.inferred_pose_t = self.get_parameter("inferred_pose_t").value
        self.traj_chosen_topic = self.get_parameter("traj_chosen_topic").value

    def cb_goal(self, msg):
        goal = self.dtype(utils.rospose_to_posetup(msg.pose))
        self.get_logger().info("Got goal")
        if self.rhctrl is not None:
            if not self.rhctrl.set_goal(goal):
                self.get_logger().error("That goal is unreachable, please choose another")
            else:
                self.get_logger().info("Goal set")
                self.goal = goal
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = 1
                marker.type = marker.ARROW
                marker.action = marker.ADD
                marker.pose = msg.pose
                marker.color.r = 1.0
                marker.color.b = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                self.goal_pub.publish(marker)

    def cb_initialpose(self, msg):
        self.init_pose = self.dtype(utils.rospose_to_posetup(msg.pose.pose))
        self.get_logger().info("Got initial pose")

        if self.debug_current_path:
            self.current_path.action = self.current_path.DELETE
            self.current_path_pub.publish(self.current_path)
            self.current_path.action = self.current_path.ADD
            self.current_path.points = []

        if self.debug_rollouts:
            if self.goal is not None:
                self.rhctrl.step(self.init_pose)
            else:
                self.get_logger().info("No goal set")

    def cb_inferred_pose(self, msg):
        if self.init_pose is not None:
            self.current_path.header.stamp = self.get_clock().now().to_msg()
            self.current_path.points.append(msg.pose.position)
            self.current_path_pub.publish(self.current_path)
        self.inferred_pose = self.dtype(utils.rospose_to_posetup(msg.pose))

    def cb_traj_chosen(self, msg):
        self.traj_chosen = list(msg.poses)

    def pub_heat_map(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 1
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.position.x = self.map_data.origin_x
        marker.pose.position.y = self.map_data.origin_y
        marker.pose.position.z = 0.0
        marker.pose.orientation = utils.angle_to_rosquaternion(self.map_data.angle)
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.scale.x = 0.5

        rospoints = []
        for i in range(150, self.map_data.width - 150, 50):
            for j in range(150, self.map_data.height - 150, 50):
                rospoints.append(self.dtype([i, j]).mul_(self.map_data.resolution))

        rospoints = torch.stack(rospoints)
        k_val = self.get_int("K")
        t_val = self.get_int("T")

        collisions = self.dtype(k_val * t_val, 3)
        for i in range(0, len(rospoints), k_val * t_val):
            end = min(len(rospoints) - i, k_val * t_val)
            collisions[:end, :2] = rospoints[i : i + end]
            col = self.rhctrl.cost.world_rep.collisions(collisions)
            for p, c in zip(collisions[:end], col[:end]):
                if c == 0:
                    marker.points.append(
                        Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                    )

        points = self.dtype(k_val, 3)
        colors = []
        for i in range(0, len(marker.points), k_val):
            end = min(len(marker.points) - i, k_val)
            points[:end, 0] = self.dtype(map(lambda p: p.x, marker.points[i : i + end]))
            points[:end, 1] = self.dtype(map(lambda p: p.y, marker.points[i : i + end]))
            c2g = self.rhctrl.cost.value_fn.get_value(points)
            colors.extend(map(float, list(c2g)[:end]))

        norm = mplcolors.Normalize(vmin=min(colors), vmax=max(colors))
        cmap = cm.get_cmap("coolwarm")

        def colorfn(cost):
            col = cmap(norm(cost))
            r, g, b, a = col[0], col[1], col[2], 1.0
            if len(col) > 3:
                a = col[3]
            return ColorRGBA(r=r, g=g, b=b, a=a)

        marker.colors = list(map(colorfn, colors))
        self.value_heat_map_pub.publish(marker)

    def viz_traj_chosen_trace_cb(self):
        if self.traj_chosen is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.traj_chosen_id
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.scale.x = 0.03
        marker.points = [pose.position for pose in self.traj_chosen]

        self.traj_chosen_pub.publish(marker)
        self.traj_chosen_id += 1

    def viz_cost_fn_cb(self):
        if self.goal is not None and self.inferred_pose is not None:
            self.rhctrl.step(self.inferred_pose)


def main(args=None):
    rclpy.init(args=args)
    node = RHCDebug(rhctensor.float_tensor())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
