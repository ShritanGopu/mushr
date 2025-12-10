#!/usr/bin/env python

"""
Converts FoxGlove ROS messages into a format compatible with the MuSHR stack.
Author: Schiffer
"""

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class NavMsgConverter(Node):
  def __init__(self) -> None:
    """
    Initialize Navigation Messages Converter.
    Attributes:
        name (string) rosnode name
    """

    super().__init__('nav_msg_converter')

    self.type = None

    # Create the subscribers
    self.pose_sub = self.create_subscription(
      self.get_parameter("~pose_topic"), PoseStamped, self.publish_pose, queue_size=100
    )
    self.type_sub = self.create_subscription(
      self.get_parameter("~type_topic"), String, self.save_type, queue_size=100
    )

    # Create the publishers
    self.goal_pub = self.create_publisher(self.get_parameter("~goal_topic"), PoseStamped, queue_size=1)
    self.car_pose_pub = self.create_publisher(self.get_parameter("~start_topic"), PoseStamped, queue_size=1)
    self.pose_estimate_pub = self.create_publisher(self.get_parameter("~estimate_topic"), PoseStamped, queue_size=1)

  def publish_pose(self, pose_msg: PoseStamped) -> None:
    """
    Take a pose stamped message and save it.
    """
    print("publishing")
    if self.type == 'goal':
      self.goal_pub.publish(pose_msg)
    elif self.type == 'estimate':
      self.pose_estimate_pub.publish(pose_msg)
    else:
      self.car_pose_pub.publish(pose_msg)
  
  def save_type(self, type_msg: String) -> None:
    self.type = type_msg.data
    print("type: " + str(self.type))
    if self.type != 'pose' and self.type != 'goal' and self.type != 'estimate':
      raise Exception(f'Invalid type detected {self.type}')

