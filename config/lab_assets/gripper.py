# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Universal Robots.

The following configuration parameters are available:

* :obj:`UR10_CFG`: The UR10 arm without a gripper.

Reference: https://github.com/ros-industrial/universal_robot
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

EZGRIPPER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/workspace/isaaclab/data_storage/ezgripper/model/model.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=[0, 1, 0],
        rot=[0.707, 0, 0, 0.707],
        joint_pos={
            "left_ezgripper_knuckle_palm_L1_1": 0.0,
            "left_ezgripper_knuckle_palm_L1_2": 0.0,
            "left_ezgripper_knuckle_L1_L2_1": 0.0,
            "left_ezgripper_knuckle_L1_L2_2": 0.0,
        },
    ),
    actuators={
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=200.0,
            damping=10.0,
        ),
    },
)
"""Configuration of EZgripper"""

BARRETT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/workspace/isaaclab/data_storage/barrett_hand/barrett_hand/model/model.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=[0, 2, 0],
        rot=[0.707, 0, 0, 0.707],
        joint_pos={
            "bh_j11_joint": 1.0,
            "bh_j21_joint": 1.0,
            "bh_j12_joint": 1.0,
            "bh_j22_joint": 1.0,
            "bh_j32_joint": 1.0,
            "bh_j13_joint": 1.0,
            "bh_j23_joint": 1.0,
            "bh_j33_joint": 1.0,
        },
    ),
    actuators={
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
    },
)
"""Configuration of Barrett hand"""

WSG32_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/workspace/isaaclab/source/usd/wsg_32/combined_wsg_32/combined_wsg_32.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint_x": 0.0,
            "joint_y": 0.0,
            "joint_z": 0.0,
            "joint_revolute": 0.0,
            "wsg_50_gripper_base_joint_gripper_left": 0.0,
            "wsg_50_gripper_base_joint_gripper_right": 0.0,
        },
    ),
    actuators={
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=0.5,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
    },
)
"""Configuration of WSG32 hand"""