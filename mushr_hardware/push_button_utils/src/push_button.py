#!/usr/bin/env python

import sys
import Jetson.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Bool
import rclpy

class PushButton(Node):

    def __init__(self):
        self.gpio_pin = self.get_parameter("push_button/gpio_pin", 31)
        self.pub_topic = self.get_parameter("push_button/pub_topic", "push_button_state")
        self.pub_rate = self.get_parameter("push_button/pub_rate", 100)

        self.state_pub = self.create_publisher(Bool, self.pub_topic, queue_size=1)

    def start(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpio_pin, GPIO.IN)

        loop_rate = self.create_rate(self.pub_rate)
        bool_msg = Bool()
        try:
            while True:
                val = GPIO.input(self.gpio_pin)

                bool_msg.data = (val < 1)
                self.state_pub.publish(bool_msg)
                loop_rate.sleep()

        finally:
            GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PushButton()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

