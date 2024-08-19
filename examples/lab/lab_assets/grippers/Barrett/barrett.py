# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Universal Robots.

The following configuration parameters are available:

* :obj:`BARRET_CFG`: The Barrett hand.

"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
import os,sys
ISAAC_TUTORIAL_PATH=os.environ['ISAAC_TUTORIAL_PATH']

##
# Configuration
##


BARRETT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_TUTORIAL_PATH}/examples/lab/lab_assets/grippers/Barrett/Barrett.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
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
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=80.0,
        ),
    },
)
