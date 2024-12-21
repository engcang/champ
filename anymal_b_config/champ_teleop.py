#!/usr/bin/env python3
#credits to: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

import math
import numpy as np
import rclpy
from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose as Pose
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class Teleop(Node):
    def __init__(self):
        super().__init__('champ_teleop')
		
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 1)
        
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 1)

        self.declare_parameter("gait/swing_height", 0)
        self.declare_parameter("gait/nominal_height", 0)
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("turn", 1.0)
        
        self.swing_height = self.get_parameter("gait/swing_height").value
        self.nominal_height = self.get_parameter("gait/nominal_height").value

        self.speed = self.get_parameter("speed").value
        self.turn = self.get_parameter("turn").value

    def joy_callback(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.velocity_publisher.publish(twist)

        calculated_roll = (not data.buttons[5]) *-data.axes[3] * 0.349066
        calculated_pitch = data.axes[4] * 0.174533
        calculated_yaw = data.buttons[5] * data.axes[3] * 0.436332
        body_pose = Pose()
        if data.axes[5] < 0:
            body_pose.position.z = data.axes[5] * 0.5
        quaternion = quaternion_from_euler(calculated_roll, calculated_pitch, calculated_yaw)
        # body_pose.orientation.x = quaternion[0]
        # body_pose.orientation.y = quaternion[1]
        # body_pose.orientation.z = quaternion[2]
        # body_pose.orientation.w = quaternion[3]
        body_pose.orientation.x = quaternion[1]
        body_pose.orientation.y = quaternion[2]
        body_pose.orientation.z = quaternion[3]
        body_pose.orientation.w = quaternion[0]

        self.pose_publisher.publish(body_pose)

if __name__ == "__main__":
    rclpy.init()
    teleop = Teleop()
    rclpy.spin(teleop)