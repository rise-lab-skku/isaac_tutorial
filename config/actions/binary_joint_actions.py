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


class BinaryJointPositionAction(BinaryJointAction):
    """Binary joint action that sets the binary action into joint position targets."""

    cfg: actions_cfg.BinaryJointPositionActionCfg
    """The configuration of the action term."""

    def apply_actions(self):
        # print(self._joint_ids)
        self._asset.set_joint_position_target(self._processed_actions, joint_ids=self._joint_ids)


class BinaryJointVelocityAction(BinaryJointAction):
    """Binary joint action that sets the binary action into joint velocity targets."""

    cfg: actions_cfg.BinaryJointVelocityActionCfg
    """The configuration of the action term."""

    def apply_actions(self):
        self._asset.set_joint_velocity_target(self._processed_actions, joint_ids=self._joint_ids)

# class BinaryJointPositionAction(BinaryJointAction):

#     cfg: actions_cfg.BinaryJointPositionActionCfg

#     def step_constraints(self):
#         # define palm joint
#         _palm_joint_ids = [6, 7]
#         palm_joint=0.5
#         palm_joint_another=None

#         _finger_rotation1 = palm_joint
#         _finger_rotation2 = palm_joint_another if palm_joint_another is not None else palm_joint
#         # rotate finger2 and finger3
#         self._asset.set_joint_position_target(torch.tensor([[_finger_rotation1, _finger_rotation2]]), joint_ids=_palm_joint_ids)
#         # return joint_positions
#         pos = self._asset._data.joint_pos[0][self._joint_ids]
#         # define driver joint; the follower joints need to satisfy constraints when grasping
#         _follower_joint_ids = [8,9,11,12,13]
#         self._asset.set_joint_position_target(torch.tensor([[pos, pos, 0.32-0.2*pos, 0.32-0.2*pos, 0.32-0.2*pos]]), joint_ids=_follower_joint_ids)


# #     def open(self, mount_gripper_id, n_joints_before, open_scale):
        
# #         # position control
# #         target_pos = open_scale*self._joint_lower + (1-open_scale)*self._joint_upper  # recalculate scale because larger joint position corresponds to smaller open width
# #         self._bullet_client.setJointMotorControl2(
# #             mount_gripper_id,
# #             self._driver_joint_id+n_joints_before,
# #             self._bullet_client.POSITION_CONTROL,
# #             targetPosition=target_pos,
# #             force=self._force
# #         )
# #         for i in range(240 * 2):
# #             pos = self.step_constraints(mount_gripper_id, n_joints_before)
# #             if np.abs(target_pos-pos)<1e-5:
# #                 break
# #             self._bullet_client.stepSimulation()
    
#     def apply_actions(self):
#         _driver_joint_id = 10
#         self._asset.set_joint_position_target(self._processed_actions, joint_ids=_driver_joint_id)
#         self.step_constraints()