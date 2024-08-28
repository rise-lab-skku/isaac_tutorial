#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__("joint_command_publisher")
        self.publisher_ = self.create_publisher(JointState, "/isaac_joint_commands", 10)

        # Create a JointState message
        self.joint_state = JointState()
        self.joint_state.name = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
            "panda_finger_joint1", "panda_finger_joint2",
        ]
        num_joints = len(self.joint_state.name)
        self.joint_state.position = [0.0] * num_joints

        # position control the robot to wiggle around each joint
        self.default_joints = np.array([0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4])
        self.motion_range = np.array([0.5] * num_joints)  # 0.5 radian
        self.time_start = time.time()

        timer_period = 0.01  # seconds (100Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        offset = np.sin(time.time() - self.time_start) * self.motion_range
        joint_position = self.default_joints + offset
        self.joint_state.position = joint_position.tolist()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    ros2_publisher = JointCommandPublisher()
    rclpy.spin(ros2_publisher)

    # Destroy the node explicitly
    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
