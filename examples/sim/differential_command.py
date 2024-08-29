import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class DifferentialDriveKinematics(Node):
    def __init__(self, wheel_radius, wheel_base):
        super().__init__('differential_drive_kinematics')
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base

        # Subscribe to cmd_vel topic to receive target velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_command', 10)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity from Twist message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate the wheel velocities using inverse kinematics
        left_wheel_velocity, right_wheel_velocity = self.inverse_kinematics(linear_velocity, angular_velocity)

        # Publish the wheel velocities as joint states
        self.publish_joint_states(left_wheel_velocity, right_wheel_velocity)

    def inverse_kinematics(self, linear_velocity, angular_velocity):
        """
        Computes the wheel velocities from the robot's linear and angular velocities.

        :param linear_velocity: Linear velocity of the robot (m/s)
        :param angular_velocity: Angular velocity of the robot (rad/s)
        :return: (left_wheel_velocity, right_wheel_velocity) in radians per second
        """
        left_wheel_velocity = (2 * linear_velocity - angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        right_wheel_velocity = (2 * linear_velocity + angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        return left_wheel_velocity, right_wheel_velocity

    def publish_joint_states(self, left_wheel_velocity, right_wheel_velocity):
        # Create JointState message
        joint_state = JointState()

        # Assuming the wheel joints are named 'left_wheel_joint' and 'right_wheel_joint'
        joint_state.name = ['joint_wheel_left', 'joint_wheel_right']
        joint_state.velocity = [left_wheel_velocity, right_wheel_velocity]
        joint_state.position = [np.nan, np.nan]
        # Populate the header with the current time
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Publish the joint state
        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)

    # Parameters for the differential drive robot
    wheel_radius = 0.14  # meters
    wheel_base = 0.413  # meters

    differential_drive_kinematics = DifferentialDriveKinematics(wheel_radius, wheel_base)

    rclpy.spin(differential_drive_kinematics)

    differential_drive_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
