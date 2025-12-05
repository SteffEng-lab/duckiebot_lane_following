#!/usr/bin/env python3

# JOSEPH

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from duckietown_msgs.msg import WheelsCmdStamped

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.declare_parameter('wheel_topic', '/wheels_driver_node/wheels_cmd')
        self.wheel_topic = self.get_parameter('wheel_topic').get_parameter_value().string_value

        self.x_m = None
        self.x_v = None
        self.gap_counter = 0
        self.gap_threshold = 5  # frames to wait before triggering a turn

        self.vanishing_sub = self.create_subscription(Point,
            '/lane_following/vanishing_point', self.vanishing_cb, 10)
        self.midpoint_sub = self.create_subscription(Point,
            '/lane_following/mid_point', self.midpoint_cb, 10)

        self.wheel_pub = self.create_publisher(WheelsCmdStamped, self.wheel_topic, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('StateMachineNode initialized')

    def vanishing_cb(self, msg):
        self.x_v = msg.x

    def midpoint_cb(self, msg):
        self.x_m = msg.x

    def control_loop(self):
        if self.x_m is None or self.x_v is None:
            self.gap_counter += 1
            if self.gap_counter >= self.gap_threshold:
                # Assume one side line is lost — decide turn direction based on last known x_m
                if self.x_m is not None and self.x_m > 0:
                    self.send_turn(right=True)
                elif self.x_m is not None and self.x_m < 0:
                    self.send_turn(right=False)
            return

        # Reset gap counter if data is valid
        self.gap_counter = 0

    def send_turn(self, right=True):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if right:
            msg.vel_left = 2.0
            msg.vel_right = 0.5
            self.get_logger().warn('Line lost → turning RIGHT')
        else:
            msg.vel_left = 0.5
            msg.vel_right = 2.0
            self.get_logger().warn('Line lost → turning LEFT')
        self.wheel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
