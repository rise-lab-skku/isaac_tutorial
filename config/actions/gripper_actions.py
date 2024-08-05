# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING
import numpy as np

import carb

import omni.isaac.lab.utils.string as string_utils
from omni.isaac.lab.assets.articulation import Articulation
from omni.isaac.lab.managers.action_manager import ActionTerm

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedEnv

    from . import actions_cfg


class BinaryJointAction(ActionTerm):
    """Base class for binary joint actions.

    This action term maps a binary action to the *open* or *close* joint configurations. These configurations are
    specified through the :class:`BinaryJointActionCfg` object. If the input action is a float vector, the action
    is considered binary based on the sign of the action values.

    Based on above, we follow the following convention for the binary action:

    1. Open action: 1 (bool) or positive values (float).
    2. Close action: 0 (bool) or negative values (float).

    The action term can mostly be used for gripper actions, where the gripper is either open or closed. This
    helps in devising a mimicking mechanism for the gripper, since in simulation it is often not possible to
    add such constraints to the gripper.
    """

    cfg: actions_cfg.BinaryJointActionCfg
    """The configuration of the action term."""

    _asset: Articulation
    """The articulation asset on which the action term is applied."""

    def __init__(self, cfg: actions_cfg.BinaryJointActionCfg, env: ManagerBasedEnv) -> None:
        # initialize the action term
        super().__init__(cfg, env)

        # resolve the joints over which the action term is applied
        self._joint_ids, self._joint_names = self._asset.find_joints(self.cfg.joint_names)
        self._num_joints = len(self._joint_ids)
        # log the resolved joint names for debugging
        carb.log_info(
            f"Resolved joint names for the action term {self.__class__.__name__}:"
            f" {self._joint_names} [{self._joint_ids}]"
        )

        # create tensors for raw and processed actions
        self._raw_actions = torch.zeros(self.num_envs, 1, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, self._num_joints, device=self.device)

        # parse open command
        self._open_command = torch.zeros(self._num_joints, device=self.device)
        index_list, name_list, value_list = string_utils.resolve_matching_names_values(
            self.cfg.open_command_expr, self._joint_names
        )
        if len(index_list) != self._num_joints:
            raise ValueError(
                f"Could not resolve all joints for the action term. Missing: {set(self._joint_names) - set(name_list)}"
            )
        self._open_command[index_list] = torch.tensor(value_list, device=self.device)

        # parse close command
        self._close_command = torch.zeros_like(self._open_command)
        index_list, name_list, value_list = string_utils.resolve_matching_names_values(
            self.cfg.close_command_expr, self._joint_names
        )
        if len(index_list) != self._num_joints:
            raise ValueError(
                f"Could not resolve all joints for the action term. Missing: {set(self._joint_names) - set(name_list)}"
            )
        self._close_command[index_list] = torch.tensor(value_list, device=self.device)

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        return 1

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    """
    Operations.
    """

    def process_actions(self, actions: torch.Tensor):
        # store the raw actions
        self._raw_actions[:] = actions
        # compute the binary mask
        if actions.dtype == torch.bool:
            # true: close, false: open
            binary_mask = actions == 0
        else:
            # true: close, false: open
            binary_mask = actions < 0
        # compute the command
        self._processed_actions = torch.where(binary_mask, self._close_command, self._open_command)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = 0.0

class BarrettHandJointControl(BinaryJointAction):

    cfg: actions_cfg.BarrettHandJointControlCfg

    def __init__(self, cfg: actions_cfg.BarrettHandJointControlCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        
        # define palm joint
        self._palm_joint_ids = [0 + self.cfg.mount_joint_num, 1 + self.cfg.mount_joint_num]
        self._palm_joint_another = None
        
        self._finger_rotation1 = self.cfg.palm_joint
        self._finger_rotation2 = self._palm_joint_another if self._palm_joint_another is not None else self.cfg.palm_joint
        
        # define driver joint
        self._driver_joint_id = 4 + self.cfg.mount_joint_num

        # define follower joint
        self._follower_joint_ids = [
            2 + self.cfg.mount_joint_num,
            3 + self.cfg.mount_joint_num, 
            5 + self.cfg.mount_joint_num, 
            6 + self.cfg.mount_joint_num,
            7 + self.cfg.mount_joint_num
        ]

    def step_constraints(self):
        # rotate finger2 and finger3
        self._asset.set_joint_position_target(torch.tensor([[self._finger_rotation1, self._finger_rotation2]]), joint_ids=self._palm_joint_ids)
        # return joint_positions
        pos = self._asset._data.joint_pos[0][self._joint_ids]
        # follower joints
        self._asset.set_joint_position_target(torch.tensor([[pos, pos, 0.32-0.2*pos, 0.32-0.2*pos, 0.32-0.2*pos]]), joint_ids=self._follower_joint_ids)

    def apply_actions(self):
        self._asset.set_joint_position_target(self._processed_actions, joint_ids=self._joint_ids)
        self.step_constraints()