# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Franka-Cabinet environment.
"""

import gymnasium as gym

from . import agents
from .franka_grasp_env import FrankaGraspEnv, FrankaGraspEnvCfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Franka-Grasp-Direct-v0",
    entry_point="lab_tasks.direct.franka_grasp:FrankaGraspEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": FrankaGraspEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.FrankaGraspPPORunnerCfg,
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
