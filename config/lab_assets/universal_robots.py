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


UR10_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/UniversalRobots/UR10/ur10_instanceable.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": -0.7071,
            "shoulder_lift_joint": -1.712,
            "elbow_joint": 1.712,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=200.0,
            stiffness=1000.0,
            damping=100.0,
        ),
    },
    attach_gripper=False,
)
"""Configuration of UR-10 arm using implicit actuator models."""

UR10_BARRETT_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        # usd_path=f"/workspace/isaaclab/data_storage/ur10_barret_single_articulation.usd",
        asset_path="/workspace/isaaclab/source/adagrasp/assets/gripper/ur10/combined_ur10_with_gripper.urdf",
        fix_base = True,
        default_drive_type = "position",
        default_drive_stiffness = 100000.00,
        default_drive_damping = 1000.00,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.712,
            "elbow_joint": 1.712,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
            "bh_j11_joint": 0.0,
            "bh_j21_joint": 0.0,
            "bh_j12_joint": 0.0,
            "bh_j22_joint": 0.0,
            "bh_j32_joint": 0.0,
            "bh_j13_joint": 0.0,
            "bh_j23_joint": 0.0,
            "bh_j33_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"],
            velocity_limit=100.0,
            effort_limit=200.0,
            stiffness=1000.0,
            damping=100.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["bh_.*"],
            velocity_limit=100.0,
            effort_limit=2.0,
            stiffness=20,
            damping=0.1,
        ),
    },
)

UR10_Surface_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/UniversalRobots/UR10/ur10_instanceable.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 0,
            "shoulder_lift_joint": -1.712,
            "elbow_joint": 1.712,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=1000.0,
            damping=40.0,
        ),
    },
    attach_gripper=True,
)
"""Configuration of UR-10 arm using suction gripper on end-effector."""
