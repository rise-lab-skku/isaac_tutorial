# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import gymnasium as gym
import os

from . import agents, ik_abs_env_cfg, ik_rel_env_cfg, joint_pos_env_cfg

##
# Register Gym environments.
##

##
# Joint Position Control
##
gym.register(
    id="Isaac-Tutorial-UR10-WSG-Grasp-Grasp-IK-Abs",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.UR10WSGCubeLiftEnvCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Tutorial-UR10-BH-Grasp-Grasp-IK-Abs",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.UR10BARRETTCubeLiftEnvCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Tutorial-UR10-Grasp",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.UR10ROBOTIQCubeLiftEnvCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Tutorial-UR10-ROBOTIQ-Grasp-IK-Abs",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.UR10ROBOTIQCubeLiftEnvCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Tutorial-UR10-ROBOTIQ-Grasp-IK-Rel",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg.UR10ROBOTIQCubeLiftEnvCfg,
    },
    disable_env_checker=True,
)
