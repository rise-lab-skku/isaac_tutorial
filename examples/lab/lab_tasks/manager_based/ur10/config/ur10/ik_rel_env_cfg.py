# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from omni.isaac.lab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from omni.isaac.lab.utils import configclass

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
# add isaac_tutorial asset directory to sys.path
#  add isaac_tutorial asset directory to sys.path
import os, sys
ISAAC_TUTORIAL_PATH=os.environ['ISAAC_TUTORIAL_PATH']
sys.path.append(os.path.join(ISAAC_TUTORIAL_PATH, 'examples/lab'))
from lab_assets.manipulators.ur10.universal_robots import UR10_CFG, UR10_SUCTION_CFG, UR10_WSG_CFG, UR10_BH_CFG, UR10_ROBOTIQ_CFG



@configclass
class UR10ROBOTIQCubeLiftEnvCfg(joint_pos_env_cfg.UR10ROBOTIQCubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = UR10_ROBOTIQ_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (franka)
        self.actions.body_joint_pos = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[".*"],
            body_name="ee_link",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        )

