# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Cartpole balancing environment.
"""

import gymnasium as gym

from . import agents
from .cartpole_env import CartpoleEnv, CartpoleEnvCfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Tutorial-Cartpole-Direct",
    entry_point="lab_tasks.direct.cartpole:CartpoleEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": CartpoleEnvCfg,
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_a2c_cfg_entry_point": f"{agents.__name__}:sb3_a2c_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
        "sb3_sac_cfg_entry_point": f"{agents.__name__}:sb3_sac_cfg.yaml",
    },
)
