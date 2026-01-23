#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

from platform import node
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
# from mushr_base import utils
from mushr_sim.fake_urg import FakeURG
from mushr_base.motion_model import KinematicCarMotionModel
from mushr_sim_interfaces.srv import CarPose
import mushr_sim.utils as utils
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

class SimulatedCar(Node):
    """
    Publishes joint and tf information about the racecar
    """

    def __init__(self, car_name, x, y, theta, map_info=None, permissible_region=None, speed_to_erpm_offset=None,
                 speed_to_erpm_gain=None, motion_model=None, sensor_model=None, tf_prefix=None):
        super().__init__("simulated_car_{}".format(car_name))
        self.car_name = car_name
        self.tf_prefix = tf_prefix
        if self.tf_prefix is None:
            self.tf_prefix = ""
        self.speed_to_erpm_offset = speed_to_erpm_offset
        self.speed_to_erpm_gain = speed_to_erpm_gain
        # The most recent transform from odom to base_footprint
        self.odom_to_base_trans = np.array([x, y], dtype=float)
        self.odom_to_base_trans = np.nan_to_num(self.odom_to_base_trans)
        self.odom_to_base_rot = np.nan_to_num(theta)
        self.odom_to_base_lock = Lock()
        self.permissible_region = permissible_region
        self.map_info = map_info

        # The most recent speed (m/s)
        self.last_speed = 0.0
        self.last_speed_lock = Lock()

        # The most recent steering angle (rad)
        self.last_steering_angle = 0.0
        self.last_steering_angle_lock = Lock()

        # Internal transform from the map to odom
        self.map_to_odom_trans = np.array([0, 0], dtype=float)
        self.map_to_odom_rot = 0
        self.map_to_odom_lock = Lock()

        # Message used to publish joint values
        self.joint_msg = JointState()
        self.joint_msg.name = [
            "front_left_wheel_throttle",
            "front_right_wheel_throttle",
            "back_left_wheel_throttle",
            "back_right_wheel_throttle",
            "front_left_wheel_steer",
            "front_right_wheel_steer",
        ]
        self.joint_msg.position = [0, 0, 0, 0, 0, 0]
        self.joint_msg.velocity = []
        self.joint_msg.effort = []

        self.fake_laser = sensor_model
        # Publishes joint values
        self.state_pub = self.create_publisher(PoseStamped, f"/{car_name}/car_pose", 1)
        self.odom_pub = self.create_publisher(Odometry, f"/{car_name}/odom", 1)

        # Publishes joint values
        self.cur_joints_pub = self.create_publisher(JointState, f"/{car_name}/joint_states", 1)

        # Subscribes to the initial pose of the car
        self.init_pose_sub = self.create_subscription(PoseStamped, "/reposition", self.init_pose_cb, 1)

        # Subscribes to info about the bldc (particularly the speed in rpm)
        self.speed_sub = self.create_subscription(VescStateStamped, f"/{car_name}/vesc/sensors/core", self.speed_cb, 1)
        
        # Subscribes to the position of the servo arm
        self.servo_sub = self.create_subscription(Float64, f"/{car_name}/vesc/sensors/servo_position_command", self.servo_cb, 1)
        self.motion_model = motion_model

    def init_pose_cb(self, msg):
        """
         init_pose_cb: Callback to capture the initial pose of the car
           msg: geometry_msg/PoseStamped containing the initial pose
         """
        # Get the pose of the car w.r.t the map in meters
        rx_trans = np.array(
            [msg.pose.position.x, msg.pose.position.y], dtype=float
        )
        rx_rot = utils.quaternion_to_angle(msg.pose.orientation)

        # Get the pose of the car w.r.t the map in pixels
        if self.map_info is not None:
            map_rx_pose = utils.world_to_map(
                (rx_trans[0], rx_trans[1], rx_rot), self.map_info
            )
            # Update the pose of the car if either bounds checking is not enabled,
            # or bounds checking is enabled but the car is in-bounds
            if not check_position_in_bounds(map_rx_pose[0], map_rx_pose[1], self.permissible_region):
                self.get_logger().warn("Requested reposition into obstacle. Ignoring.")
                return

        with self.odom_to_base_lock:
            # Move the vehicle by updating the odom->base transform
            self.odom_to_base_trans = rx_trans
            self.odom_to_base_rot = rx_rot

    def speed_cb(self, msg):
        """
        Callback to capture the speed of the car
          msg: vesc_msgs/VescStateStamped message containing the speed of the car (rpm)
        """
        self.last_speed_lock.acquire()
        self.last_speed = (msg.state.speed - self.speed_to_erpm_offset) / self.speed_to_erpm_gain
        self.last_speed_lock.release()

    def servo_cb(self, msg):
        """
        Callback to capture the steering angle of the car
         msg: std_msgs/Float64 message containing the servo value
        """
        with self.last_steering_angle_lock:
            self.last_steering_angle = (msg.data - self.motion_model.steering_to_servo_offset) / self.motion_model.steering_to_servo_gain

    def reposition(self, x, y, theta):
        rx_trans = np.array(
            [x, y], dtype=float
        )
        rx_rot = theta

        # Get the pose of the car w.r.t the map in pixels
        if self.map_info is not None:
            map_rx_pose = utils.world_to_map(
                (rx_trans[0], rx_trans[1], rx_rot), self.map_info
            )
            # Update the pose of the car if either bounds checking is not enabled,
            # or bounds checking is enabled but the car is in-bounds
            if not check_position_in_bounds(map_rx_pose[0], map_rx_pose[1], self.permissible_region):
                self.get_logger().warn("Requested reposition into obstacle. Ignoring.")
                return

        with self.odom_to_base_lock:
            # Move the vehicle by updating the odom->base transform
            self.odom_to_base_trans = rx_trans
            self.odom_to_base_rot = rx_rot

        with self.odom_to_base_lock:
            self.odom_to_base_trans = np.array([x, y], dtype=float)
            self.odom_to_base_rot = theta

    def simulate(self, dt, now):

        # Add noise to the speed
        with self.last_speed_lock:
            v = self.last_speed

        # Add noise to the steering angle
        with self.last_steering_angle_lock:
            delta = self.last_steering_angle

        with self.odom_to_base_lock:
            # Apply kinematic car model to the previous pose
            new_pose = np.array(
                [
                    self.odom_to_base_trans[0],
                    self.odom_to_base_trans[1],
                    self.odom_to_base_rot,
                ],
                dtype=float,
            )

            state_changes, joint_changes = self.motion_model.apply_motion_model(new_pose[np.newaxis, ...],
                                                                             np.array([[v, delta]]), dt)

            state_changes = state_changes.squeeze()
            joint_changes = joint_changes.squeeze()
            in_bounds = True
            if self.permissible_region is not None:
                # Compute the new pose w.r.t the map in meters
                new_map_pose = np.zeros(3, dtype=float)
                new_map_pose[0] = self.map_to_odom_trans[0] + (
                        new_pose[0] * np.cos(self.map_to_odom_rot)
                        - new_pose[1] * np.sin(self.map_to_odom_rot)
                )
                new_map_pose[1] = self.map_to_odom_trans[1] + (
                        new_pose[0] * np.sin(self.map_to_odom_rot)
                        + new_pose[1] * np.cos(self.map_to_odom_rot)
                )
                new_map_pose[2] = self.map_to_odom_rot + new_pose[2]

                # Get the new pose w.r.t the map in pixels
                if self.permissible_region is not None:
                    new_map_pose = utils.world_to_map(new_map_pose, self.map_info)
                    in_bounds = check_position_in_bounds(new_map_pose[0], new_map_pose[1], self.permissible_region)

            if in_bounds:
                # Update pose of base_footprint w.r.t odom
                self.odom_to_base_trans[0] = new_pose[0]
                self.odom_to_base_trans[1] = new_pose[1]
                self.odom_to_base_rot = new_pose[2]

                # Update joint values
                self.joint_msg.position[0] += joint_changes[0]
                self.joint_msg.position[1] += joint_changes[1]
                self.joint_msg.position[2] += joint_changes[0]
                self.joint_msg.position[3] += joint_changes[1]
                self.joint_msg.position[4] = joint_changes[2]
                self.joint_msg.position[5] = joint_changes[3]

                # Clip all joint angles
                for i in range(len(self.joint_msg.position)):
                    self.joint_msg.position[i] = utils.wrap_angle(self.joint_msg.position[i])
            else:
                self.get_logger().warn("Not in bounds")

            # Publish the joint states
            self.joint_msg.header.stamp = now.to_msg()
            self.cur_joints_pub.publish(self.joint_msg)

            t = utils.make_transform_msg(self.odom_to_base_trans, self.odom_to_base_rot,
                                         self.tf_prefix + "ground_truth_base_footprint", "map")

            # Tell the laser where we are
            # rospy.logerr_throttle(1,t)
            # rospy.logerr_throttle(1, new_pose)
            self.fake_laser.transform = t.transform
            self.transform = t
            self.publish_updated_transforms(self.transform, state_changes)

        # Publish current state as a PoseStamped topic

    def publish_updated_transforms(self, transform, changes):
        cur_pose = PoseStamped()
        cur_pose.header.frame_id = "map"

        t = rclpy.time.Time.from_msg(transform.header.stamp)
        t = t - rclpy.duration.Duration(seconds=0.5)
        cur_pose.header.stamp = t.to_msg()

        # for visualization purposes, delay header stamp
        cur_pose.pose.position.x = (
                self.odom_to_base_trans[0] + self.map_to_odom_trans[0]
        )
        cur_pose.pose.position.y = (
                self.odom_to_base_trans[1] + self.map_to_odom_trans[1]
        )
        cur_pose.pose.position.z = 0.0
        rot = self.odom_to_base_rot + self.map_to_odom_rot
        cur_pose.pose.orientation = utils.angle_to_quaternion(rot)
        self.state_pub.publish(cur_pose)

        odom_msg = Odometry()
        odom_msg.header.stamp = transform.header.stamp
        odom_msg.header.frame_id = self.tf_prefix + "odom"
        odom_msg.pose.pose.position.x = transform.transform.translation.x
        odom_msg.pose.pose.position.y = transform.transform.translation.y
        odom_msg.pose.pose.position.z = transform.transform.translation.z
        odom_msg.pose.pose.orientation = transform.transform.rotation

        odom_msg.child_frame_id = self.tf_prefix + "base_link"
        odom_msg.twist.twist.linear.x = changes[0]
        odom_msg.twist.twist.linear.y = changes[1]
        odom_msg.twist.twist.angular.z = changes[2]

        self.odom_pub.publish(odom_msg)

class MushrSim(Node):
    """
    __init__: Initialize params, publishers, subscribers, and timer
    """

    def __init__(self):
        super().__init__("mushr_sim")
        self.declare_parameter("use_tf_prefix", True)
        self.use_tf_prefix = bool(self.get_parameter("use_tf_prefix").value)

        # speed (rpm) = SPEED_TO_ERPM_OFFSET + SPEED_TO_ERPM_GAIN * speed (m/s)
        self.declare_parameter("vesc.speed_to_erpm_offset", 0.0)
        self.declare_parameter("vesc.speed_to_erpm_gain", 4614.0)
        self.speed_to_erpm_offset = float(self.get_parameter("vesc.speed_to_erpm_offset").value)
        self.speed_to_erpm_gain = float(self.get_parameter("vesc.speed_to_erpm_gain").value)

        # servo angle = STEERING_TO_SERVO_OFFSET + STEERING_TO_SERVO_GAIN * steering_angle (rad)
        self.declare_parameter("vesc.steering_angle_to_servo_offset", 0.5304)
        self.declare_parameter("vesc.steering_angle_to_servo_gain", -1.2135)
        steering_to_servo_offset = float(self.get_parameter("vesc.steering_angle_to_servo_offset").value)
        steering_to_servo_gain = float(self.get_parameter("vesc.steering_angle_to_servo_gain").value)

        # Car geometry
        self.declare_parameter("vesc.chassis_length", 0.33)
        self.declare_parameter("vesc.wheelbase", 0.25)
        self.car_length = float(self.get_parameter("vesc.chassis_length").value)
        self.car_width = float(self.get_parameter("vesc.wheelbase").value)

        # Wheel radius (constant here; make a param if you want)
        self.car_wheel_radius = 0.0976 / 2.0

        # Rate at which to publish joints and tf
        self.declare_parameter("update_rate", 20.0)
        self.update_rate = float(self.get_parameter("update_rate").value)

        # Motion model parameters
        self.declare_parameter("motion_model.vel_bias", 0.0)
        self.declare_parameter("motion_model.vel_std", 0.0001)
        self.declare_parameter("motion_model.delta_bias", 0.0)
        self.declare_parameter("motion_model.delta_std", 0.000001)
        self.declare_parameter("motion_model.x_bias", 0.0)
        self.declare_parameter("motion_model.x_std", 0.0000001)
        self.declare_parameter("motion_model.x_vel_std", 0.001)
        self.declare_parameter("motion_model.y_bias", 0.0)
        self.declare_parameter("motion_model.y_std", 0.000001)
        self.declare_parameter("motion_model.y_vel_std", 0.001)
        self.declare_parameter("motion_model.theta_bias", 0.0)
        self.declare_parameter("motion_model.theta_std", 0.000001)

        # FakeURG sensor parameters
        self.declare_parameter("fake_urg.update_rate", 10.0)
        self.declare_parameter("fake_urg.theta_discretization", 656)
        self.declare_parameter("fake_urg.z_hit", 0.8)
        self.declare_parameter("fake_urg.z_short", 0.03)
        self.declare_parameter("fake_urg.z_max", 0.16)
        self.declare_parameter("fake_urg.z_blackout_max", 50)
        self.declare_parameter("fake_urg.z_rand", 0.01)
        self.declare_parameter("fake_urg.z_sigma", 0.03)
        self.declare_parameter("fake_urg.tf_prefix", "")

        # Extract motion model parameters
        self.motion_params = {
            "vel_bias": float(self.get_parameter("motion_model.vel_bias").value),
            "vel_std": float(self.get_parameter("motion_model.vel_std").value),
            "delta_bias": float(self.get_parameter("motion_model.delta_bias").value),
            "delta_std": float(self.get_parameter("motion_model.delta_std").value),
            "x_bias": float(self.get_parameter("motion_model.x_bias").value),
            "x_std": float(self.get_parameter("motion_model.x_std").value),
            "x_vel_std": float(self.get_parameter("motion_model.x_vel_std").value),
            "y_bias": float(self.get_parameter("motion_model.y_bias").value),
            "y_std": float(self.get_parameter("motion_model.y_std").value),
            "y_vel_std": float(self.get_parameter("motion_model.y_vel_std").value),
            "theta_bias": float(self.get_parameter("motion_model.theta_bias").value),
            "theta_std": float(self.get_parameter("motion_model.theta_std").value),
            "steering_to_servo_offset": steering_to_servo_offset,
            "steering_to_servo_gain": steering_to_servo_gain,
            "car_length": self.car_length,
            "car_width": self.car_width,
            "car_wheel_radius": self.car_wheel_radius,
        }

        # Extract FakeURG sensor parameters
        self.sensor_params = {
            "update_rate": float(self.get_parameter("fake_urg.update_rate").value),
            "theta_discretization": int(self.get_parameter("fake_urg.theta_discretization").value),
            "z_hit": float(self.get_parameter("fake_urg.z_hit").value),
            "z_short": float(self.get_parameter("fake_urg.z_short").value),
            "z_max": float(self.get_parameter("fake_urg.z_max").value),
            "z_blackout_max": int(self.get_parameter("fake_urg.z_blackout_max").value),
            "z_rand": float(self.get_parameter("fake_urg.z_rand").value),
            "z_sigma": float(self.get_parameter("fake_urg.z_sigma").value),
            "tf_prefix": str(self.get_parameter("fake_urg.tf_prefix").value),
        }

        # The map and map params
        self.permissible_region = None
        self.map_info = None


        self.map_service_name = '/map_server/map'

        self.map_client = self.create_client(GetMap, self.map_service_name)

        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map Server not available, waiting again...')

        def get_map():
            req = GetMap.Request()
            future = self.map_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                
                # Numpy array of dimension (future.result().map.info.height, future.result().map.info.width),
                array_255 = np.array(future.result().map.data).reshape((future.result().map.info.height, future.result().map.info.width))
                permissible_region = np.zeros_like(array_255, dtype=bool)
                permissible_region[array_255 == 0] = 1  # With values 0: not permissible, 1: permissible 
                
                return permissible_region, future.result().map.info, future.result().map.data
            else:
                self.get_logger().error('Service call failed %r' % (future.exception(),))
                return None, None
        # Get the map
        self.permissible_region, self.map_info, self.raw_map_msg = get_map()

        # Publishes joint messages
        self.br = tf2_ros.TransformBroadcaster(self)

        # Duration param controls how often to publish default map to odom tf
        # if no other nodes are publishing it
        self.tf_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._cars = []
        
        self._car_reposition_srv = self.create_service(CarPose, "reposition", self._car_reposition_cb)
        self._car_spawn_srv = self.create_service(CarPose, "spawn", self.spawn_car)

        self.last_stamp = None
        # Timer to updates joints and tf
        period = 1.0 / self.update_rate
        self.update_timer = self.create_timer(period, self.simulate_cb)

        self.default_motion_model = KinematicCarMotionModel(**self.motion_params)

        # Declare and read list of car names
        self.declare_parameter("car_names", [""])
        # pass in the list of car names in logger

        car_names = list(self.get_parameter("car_names").value)

        for car_name in car_names:
            # Per-car initial pose params
            self.declare_parameter(f"{car_name}.initial_x", 0.0)
            self.declare_parameter(f"{car_name}.initial_y", 0.0)
            self.declare_parameter(f"{car_name}.initial_theta", 0.0)

            initial_x = float(self.get_parameter(f"{car_name}.initial_x").value)
            initial_y = float(self.get_parameter(f"{car_name}.initial_y").value)
            initial_theta = float(self.get_parameter(f"{car_name}.initial_theta").value)

            request = CarPose.Request()
            request.car_name = car_name
            request.x = initial_x
            request.y = initial_y
            request.theta = initial_theta
            response = CarPose.Response()
            self.spawn_car(request, car_name, initial_x, initial_y, initial_theta, response)


    def spawn_car(self, request, car_name, x, y, theta, response):
        response.success = False
        if any(map(lambda car: car.car_name == car_name, self._cars)):
            return False
        sensor_params = self.sensor_params.copy()
        car_tf_prefix = car_name + "/" if self.use_tf_prefix else ""

        sensor_params["tf_prefix"] = car_tf_prefix
        try:
            transform = self.tf_buffer.lookup_transform(
                        car_tf_prefix + "base_link",
                        car_tf_prefix + "laser_link",
                        rclpy.time.Time(),                     # latest available is usually best here
                        timeout=rclpy.duration.Duration(seconds=10.0),
                        )
            # Drop stamp header
            transform = transform.transform

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn("Failed to spawn new car named '{}' because no TF information was found".format(car_name))
            return False

        sensor = FakeURG(self.raw_map_msg, topic_namespace=car_name + "/", x_offset=transform.translation.x,
                         **sensor_params)
        
        new_car = SimulatedCar(car_name, x, y, theta, speed_to_erpm_gain=self.speed_to_erpm_gain,
                               speed_to_erpm_offset=self.speed_to_erpm_offset,
                               permissible_region=self.permissible_region, map_info=self.map_info, sensor_model=sensor,
                               motion_model=self.default_motion_model, tf_prefix=car_tf_prefix)
        self._cars.append(new_car)

        response.success = True

        return response

    def simulate_cb(self):
        """
        Callback occurring at a rate of self.UPDATE_RATE. Updates the car joint
                  angles and tf of the base_footprint w.r.t odom. Also publishes robot state as a PoseStamped msg.
          event: Information about when this callback occurred
        """
        now = self.get_clock().now()
        # Get the time since the last update
        if self.last_stamp is None:
            self.last_stamp = now
        dt = (now - self.last_stamp).nanoseconds / 1e9

        # NOTE(nickswalker5-6-21): There's more stuff that could optionally
        # happen here. All the state updates could be calculated at once
        # because the motion model is vectorized (ROS operations would
        # need to be looped over after). Collision checking between
        # vehicles.
        for car in self._cars:
            car.simulate(dt, now)

            # Publish the tf from odom to base_footprint
            self.br.sendTransform(car.transform)

        self.last_stamp = now

    def _car_reposition_cb(self, request, response):
        response.success = False
        for car in self._cars:
            if car.car_name == request.car_name:
                car.reposition(request.x, request.y, request.theta)
                response.success = True
                break
        return response


def check_position_in_bounds(x, y, permissible_region):
    if permissible_region is None:
        return True
    return (
            0 <= x < permissible_region.shape[1] and \
            0 <= y < permissible_region.shape[0] and \
            permissible_region[
                int(y + 0.5), int(x + 0.5)
            ]
    )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        rclpy.spin(MushrSim())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()