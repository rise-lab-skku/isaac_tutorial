#!/usr/bin/env python3
"""
ros2 run isaac_tutorials pose_goal_minimal_example
"""

from math import cos, sin
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    panda_joint_names = [
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
        # "panda_finger_joint1", "panda_finger_joint2",
    ]
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda_joint_names,
        base_link_name="panda_link0",
        end_effector_name="panda_hand",
        group_name="panda_arm",
        # group_name="panda_arm_hand",
        callback_group=callback_group,
    )
    # Default planning pipeline and planner:
    #   moveit2.pipeline_id = "ompl"
    #   moveit2.planner_id = "RRTConnectkConfigDefault"
    moveit2.pipeline_id = "pilz_industrial_motion_planner"
    moveit2.planner_id = "PTP"
    # "PTP": https://moveit.picknik.ai/humble/doc/examples/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html#the-ptp-motion-command

    # # Set parameters for cartesian planning
    moveit2.cartesian_avoid_collisions = False
    moveit2.cartesian_jump_threshold = 0.0

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # =============== INITIALIZE PANDA WITH JOINT COMMANDS ===============

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.2
    moveit2.max_acceleration = 0.2

    # joint1: lower="-2.9671" upper="2.9671"
    # joint2: lower="-1.8326" upper="1.8326"
    # joint3: lower="-2.9671" upper="2.9671"
    # joint4: lower="-3.1416" upper="-0.0873"
    # joint5: lower="-2.9671" upper="2.9671"
    # joint6: lower="-0.0873" upper="3.8223"
    # joint7: lower="-2.9671" upper="2.9671"
    # finger1: lower="0.0" (close) upper="0.04" (open)
    # finger2: lower="0.0" (close) upper="0.04" (open)
    ready_joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    node.get_logger().info(f"Moving to {{joint_positions: {list(ready_joint_positions)}}}")
    moveit2.move_to_configuration(ready_joint_positions)
    moveit2.wait_until_executed()
    node.create_rate(2.0).sleep()

    # =============== BEGIN YOUR CODE HERE (POSE COMMANDS) ===============
    node.get_logger().info("Moving to pose goal!")
    # Read TF tree: ros2 run tf2_ros tf2_echo [source_frame] [target_frame]
    # Example: ros2 run tf2_ros tf2_echo panda_link0 panda_hand
    #
    # ready_position = [0.307, 0.000, 0.590]
    # ready_quat_xyzw = [1.000, 0.000, 0.000, 0.000]

    # Example triangle
    goal_positions = [
        [0.307, 0.000, 0.590],
        [0.500, 0.150, 0.590],
        [0.500, -0.150, 0.590],
    ]
    goal_quat_xyzw = [1.0, 0.0, 0.0, 0.0]
    i = 0
    max_i = len(goal_positions) - 1

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.2
    moveit2.max_acceleration = 0.15

    # Move to goal position
    while rclpy.ok():
        goal_p = goal_positions[i]
        node.get_logger().info(f"Moving to [{i}] {{position: {goal_p}, quat_xyzw: {goal_quat_xyzw}}}")

        # For cartesian plans, the plan is rejected if the fraction of the path that was completed
        # is less than `cartesian_fraction_threshold`. (range: [0.0, 1.0], 1.0 means 100% reached)
        moveit2.move_to_pose(
            position=goal_p,
            quat_xyzw=goal_quat_xyzw,
            cartesian=False,
            cartesian_max_step=0.002,
            cartesian_fraction_threshold=0.0,
        )
        # Request in service: /compute_cartesian_path [moveit_msgs/srv/GetCartesianPath]
        res = moveit2.wait_until_executed()

        if res == True:
            # Update goal position
            i = 0 if i == max_i else i + 1
            node.create_rate(2.0).sleep()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
