from isaacgym import gymapi
from isaacgym import gymtorch
from isaacgym.torch_utils import *
import os
import torch
import numpy as np

from utils.torch_jit_utils import quat_mul, to_torch, tensor_clamp

@torch.jit.script
def axisangle2quat(vec, eps=1e-6):
    """
    Converts scaled axis-angle to quat.
    Args:
        vec (tensor): (..., 3) tensor where final dim is (ax,ay,az) axis-angle exponential coordinates
        eps (float): Stability value below which small values will be mapped to 0

    Returns:
        tensor: (..., 4) tensor where final dim is (x,y,z,w) vec4 float quaternion
    """
    # type: (Tensor, float) -> Tensor
    # store input shape and reshape
    input_shape = vec.shape[:-1]
    vec = vec.reshape(-1, 3)

    # Grab angle
    angle = torch.norm(vec, dim=-1, keepdim=True)

    # Create return array
    quat = torch.zeros(torch.prod(torch.tensor(input_shape)), 4, device=vec.device)
    quat[:, 3] = 1.0

    # Grab indexes where angle is not zero an convert the input to its quaternion form
    idx = angle.reshape(-1) > eps
    quat[idx, :] = torch.cat([
        vec[idx, :] * torch.sin(angle[idx, :] / 2.0) / angle[idx, :],
        torch.cos(angle[idx, :] / 2.0)
    ], dim=-1)

    # Reshape and return output
    quat = quat.reshape(list(input_shape) + [4, ])
    return quat

class Franka:
    def __init__(self, args):
        self.args = args
        self.device = self.args.sim_device
        self.num_envs = self.args.num_envs

        # configure parameters
        self.max_episode_length = 300
        self.action_scale = 7.5
        self.dof_vel_scale = 0.1
        self.start_position_noise = 0.0
        self.start_rotation_noise = 0.0
        self.franka_position_noise = 0.0
        self.franka_rotation_noise = 0.0
        self.franka_dof_noise = 0
        self.aggregate_mode = 3
        self.dt = 1/60.
        self.force_torque_obs_scale = 10.0  # scale factor 

        # reward settings
        self.dof_vel_scale = 0.1
        self.dist_reward_scale = 2.0
        self.rot_reward_scale = 0.5
        self.finger_dist_reward_scale = 5.0
        self.action_penalty_scale = 0.01

        # Create dicts to pass to reward function
        self.reward_settings = {
            "r_dist_scale": 0.1,
            "r_lift_scale": 1.5,
            "r_grasp_scale": 16.0,
        }

        # dimensions of observation and action space
        # obs space = cubeA_pose (7) + cubeA_to_eef_pos + eef_pose (7) + q_gripper (2) + joint pos (7)
        self.numObservations = 26
        # actions include: joint pos (7) + gripper pos (2)
        self.numActions = 9

        # Values to be filled in at runtime
        self.states = {}                        # will be dict filled with relevant states to use for reward calculation
        self.handles = {}                       # will be dict mapping names to relevant sim handles
        self.num_dofs = None                    # Total number of DOFs per env
        self.actions = None                     # Current actions to be deployed
        self._init_cubeA_state = None           # Initial state of cubeA for the current env
        self._cubeA_state = None                # Current state of cubeA for the current env
        self._cubeA_id = None                   # Actor ID corresponding to cubeA for a given env

        # create some wrapper tensors for different slices
        self._root_state = None             # State of root body        (n_envs, 13)
        self._dof_state = None  # State of all joints       (n_envs, n_dof)
        self._q = None  # Joint positions           (n_envs, n_dof)
        self._qd = None                     # Joint velocities          (n_envs, n_dof)
        self._rigid_body_state = None  # State of all rigid bodies             (n_envs, n_bodies, 13)
        self._contact_forces = None     # Contact forces in sim
        self._eef_state = None  # end effector state (at grasping point)
        self._eef_lf_state = None  # end effector state (at left fingertip)
        self._eef_rf_state = None  # end effector state (at left fingertip)
        self._arm_control = None  # Tensor buffer for controlling arm
        self._gripper_control = None  # Tensor buffer for controlling gripper
        self._pos_control = None            # Position actions
        self._franka_joint_limits = None        # Actuator position limits for franka
        self._global_indices = None         # Unique indices corresponding to all envs in flattened array

        # Franka defaults
        self.franka_default_dof_pos = to_torch(
            [0, 0.6063, 0, -2.6180, 0, 2.9416, 0.7854, 0.035, 0.035], device=self.device
        )


        # initialise sim and envs
        self.create_sim(args)
        self.create_envs()

        # generate viewer for visualisation
        if not self.args.headless:
            self.viewer = self.create_viewer()

        # step simulation to initialise tensor buffers
        self.gym.prepare_sim(self.sim)

        self.reset()
        # Refresh tensors
        self._refresh()

    def create_sim(self, args):
        # configure sim (gravity is pointing down)
        sim_params = gymapi.SimParams()
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
        sim_params.dt = self.dt
        sim_params.substeps = 1
        sim_params.use_gpu_pipeline = True

        # set simulation parameters (we use PhysX engine by default, these parameters are from the example file)
        sim_params.physx.num_position_iterations = 12
        sim_params.physx.num_velocity_iterations = 1
        sim_params.physx.rest_offset = 0.0
        sim_params.physx.contact_offset = 0.005
        sim_params.physx.use_gpu = True
        sim_params.physx.num_subscenes = 4
        sim_params.physx.num_threads = 4


        # allocate buffers
        self.obs_buf = torch.zeros((self.args.num_envs, self.numObservations), device=self.args.sim_device)
        self.reward_buf = torch.zeros(self.args.num_envs, device=self.args.sim_device)
        self.reset_buf = torch.ones(self.args.num_envs, device=self.args.sim_device, dtype=torch.long)
        self.progress_buf = torch.zeros(self.args.num_envs, device=self.args.sim_device, dtype=torch.long)

        # acquire gym interface
        self.gym = gymapi.acquire_gym()
        self.sim = self.gym.create_sim(args.compute_device_id, args.graphics_device_id, gymapi.SIM_PHYSX, sim_params)

    def create_envs(self):
        # add ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        self.gym.add_ground(self.sim, plane_params)

        # define environment space (for visualisation)
        spacing = 1.5
        lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        upper = gymapi.Vec3(spacing, spacing, spacing)
        num_per_row = int(np.sqrt(self.args.num_envs))

        # add franka asset
        asset_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../urdf')
        asset_file = "franka_description/robots/franka_panda_gripper.urdf"
        asset_options = gymapi.AssetOptions()
        asset_options.flip_visual_attachments = True
        asset_options.fix_base_link = True
        asset_options.collapse_fixed_joints = False
        asset_options.disable_gravity = True
        asset_options.thickness = 0.001
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
        asset_options.use_mesh_materials = True

        franka_asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)

        franka_dof_stiffness = to_torch([400, 400, 400, 400, 400, 400, 400, 1.0e6, 1.0e6], dtype=torch.float, device=self.device)
        franka_dof_damping = to_torch([80, 80, 80, 80, 80, 80, 80, 1.0e2, 1.0e2], dtype=torch.float, device=self.device)

        # Create table asset
        table_pos = [0.0, 0.0, 1.0]
        table_thickness = 0.05
        table_opts = gymapi.AssetOptions()
        table_opts.fix_base_link = True
        table_asset = self.gym.create_box(self.sim, *[1.2, 1.2, table_thickness], table_opts)

        # Create table stand asset
        table_stand_height = 0.1
        table_stand_pos = [-0.5, 0.0, 1.0 + table_thickness / 2 + table_stand_height / 2]
        table_stand_opts = gymapi.AssetOptions()
        table_stand_opts.fix_base_link = True
        table_stand_asset = self.gym.create_box(self.sim, *[0.2, 0.2, table_stand_height], table_opts)

        self.cubeA_size = 0.050

        # Create cubeA asset
        cubeA_opts = gymapi.AssetOptions()
        cubeA_asset = self.gym.create_box(self.sim, *([self.cubeA_size] * 3), cubeA_opts)
        cubeA_color = gymapi.Vec3(0.6, 0.1, 0.0)

        self.num_franka_bodies = self.gym.get_asset_rigid_body_count(franka_asset)
        self.num_franka_dofs = self.gym.get_asset_dof_count(franka_asset)
        print("num franka bodies: ", self.num_franka_bodies)
        print("num franka dofs: ", self.num_franka_dofs)

        # set franka dof properties
        franka_dof_props = self.gym.get_asset_dof_properties(franka_asset)
        self.franka_dof_lower_limits = []
        self.franka_dof_upper_limits = []
        for i in range(self.num_franka_dofs):
            franka_dof_props['driveMode'][i] = gymapi.DOF_MODE_POS
            franka_dof_props['stiffness'][i] = franka_dof_stiffness[i]
            franka_dof_props['damping'][i] = franka_dof_damping[i]

            self.franka_dof_lower_limits.append(franka_dof_props['lower'][i])
            self.franka_dof_upper_limits.append(franka_dof_props['upper'][i])

        self.franka_dof_lower_limits = to_torch(self.franka_dof_lower_limits, device=self.device)
        self.franka_dof_upper_limits = to_torch(self.franka_dof_upper_limits, device=self.device)
        self.franka_dof_speed_scales = torch.ones_like(self.franka_dof_lower_limits)
        self.franka_dof_speed_scales[[7, 8]] = 0.1
        franka_dof_props['effort'][7] = 200
        franka_dof_props['effort'][8] = 200

        # Define start pose for franka
        franka_start_pose = gymapi.Transform()
        franka_start_pose.p = gymapi.Vec3(-0.45, 0.0, 1.0 + table_thickness / 2 + table_stand_height)
        franka_start_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)

        # Define start pose for table
        table_start_pose = gymapi.Transform()
        table_start_pose.p = gymapi.Vec3(*table_pos)
        table_start_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        self._table_surface_pos = np.array(table_pos) + np.array([0, 0, table_thickness / 2])
        self.reward_settings["table_height"] = self._table_surface_pos[2]

        # Define start pose for table stand
        table_stand_start_pose = gymapi.Transform()
        table_stand_start_pose.p = gymapi.Vec3(*table_stand_pos)
        table_stand_start_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)

        # Define start pose for cubes (doesn't really matter since they're get overridden during reset() anyways)
        cubeA_start_pose = gymapi.Transform()
        cubeA_start_pose.p = gymapi.Vec3(0.0, 0.0, 0.0)
        cubeA_start_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)

        # compute aggregate size
        num_franka_bodies = self.gym.get_asset_rigid_body_count(franka_asset)
        num_franka_shapes = self.gym.get_asset_rigid_shape_count(franka_asset)
        max_agg_bodies = num_franka_bodies + 3     # 1 for table, table stand, cubeA
        max_agg_shapes = num_franka_shapes + 3     # 1 for table, table stand, cubeA

        self.frankas = []
        self.envs = []

        # Create environments
        print(f'Creating {self.args.num_envs} environments.')
        for i in range(self.num_envs):
            # create env instance
            env_ptr = self.gym.create_env(self.sim, lower, upper, num_per_row)

            # Create actors and define aggregate group appropriately depending on setting
            # NOTE: franka should ALWAYS be loaded first in sim!
            if self.aggregate_mode >= 3:
                self.gym.begin_aggregate(env_ptr, max_agg_bodies, max_agg_shapes, True)

            # Create franka
            # Potentially randomize start pose
            if self.franka_position_noise > 0:
                rand_xy = self.franka_position_noise * (-1. + np.random.rand(2) * 2.0)
                franka_start_pose.p = gymapi.Vec3(-0.45 + rand_xy[0], 0.0 + rand_xy[1],
                                                1.0 + table_thickness / 2 + table_stand_height)
            if self.franka_rotation_noise > 0:
                rand_rot = torch.zeros(1, 3)
                rand_rot[:, -1] = self.franka_rotation_noise * (-1. + np.random.rand() * 2.0)
                new_quat = axisangle2quat(rand_rot).squeeze().numpy().tolist()
                franka_start_pose.r = gymapi.Quat(*new_quat)
            franka_actor = self.gym.create_actor(env_ptr, franka_asset, franka_start_pose, "franka", i, 0, 0)
            self.gym.set_actor_dof_properties(env_ptr, franka_actor, franka_dof_props)

            if self.aggregate_mode == 2:
                self.gym.begin_aggregate(env_ptr, max_agg_bodies, max_agg_shapes, True)

            # Create table
            table_actor = self.gym.create_actor(env_ptr, table_asset, table_start_pose, "table", i, 1, 0)
            table_stand_actor = self.gym.create_actor(env_ptr, table_stand_asset, table_stand_start_pose, "table_stand", i, 1, 0)

            if self.aggregate_mode == 1:
                self.gym.begin_aggregate(env_ptr, max_agg_bodies, max_agg_shapes, True)

            # Create cubes
            self._cubeA_id = self.gym.create_actor(env_ptr, cubeA_asset, cubeA_start_pose, "cubeA", i, 2, 0)
            # Set colors
            self.gym.set_rigid_body_color(env_ptr, self._cubeA_id, 0, gymapi.MESH_VISUAL, cubeA_color)

            if self.aggregate_mode > 0:
                self.gym.end_aggregate(env_ptr)

            self.num_actor = self.gym.get_actor_count(env_ptr)  # get number of actors in an environment

            # Store the created env pointers
            self.envs.append(env_ptr)
            self.frankas.append(franka_actor)


        # Setup init state buffer
        self._init_cubeA_state = torch.zeros(self.num_envs, 13, device=self.device)

        # Setup data
        self.init_data()

    def init_data(self):
        # Setup sim handles
        env_ptr = self.envs[0]
        franka_handle = 0
        self.handles = {
            # Franka
            "hand": self.gym.find_actor_rigid_body_handle(env_ptr, franka_handle, "panda_hand"),
            "leftfinger_tip": self.gym.find_actor_rigid_body_handle(env_ptr, franka_handle, "panda_leftfinger_tip"),
            "rightfinger_tip": self.gym.find_actor_rigid_body_handle(env_ptr, franka_handle, "panda_rightfinger_tip"),
            "grip_site": self.gym.find_actor_rigid_body_handle(env_ptr, franka_handle, "panda_grip_site"),
            # Cubes
            "cubeA_body_handle": self.gym.find_actor_rigid_body_handle(self.envs[0], self._cubeA_id, "box"),
        }

        # Get total DOFs
        self.num_dofs = self.gym.get_sim_dof_count(self.sim) // self.num_envs

        # Get gym GPU state tensors
        _actor_root_state_tensor = self.gym.acquire_actor_root_state_tensor(self.sim)
        _dof_state_tensor = self.gym.acquire_dof_state_tensor(self.sim)
        _rigid_body_state_tensor = self.gym.acquire_rigid_body_state_tensor(self.sim)
        _net_cf = self.gym.acquire_net_contact_force_tensor(self.sim)

        self._root_state = gymtorch.wrap_tensor(_actor_root_state_tensor).view(self.num_envs, -1, 13)
        self._dof_state = gymtorch.wrap_tensor(_dof_state_tensor).view(self.num_envs, -1, 2)
        self._rigid_body_state = gymtorch.wrap_tensor(_rigid_body_state_tensor).view(self.num_envs, -1, 13)
        self._net_cf = gymtorch.wrap_tensor(_net_cf).view(self.num_envs, -1, 3)

        # create some wrapper tensors for different slices
        self._q = self._dof_state[..., 0]
        self._qd = self._dof_state[..., 1]
        self._eef_state = self._rigid_body_state[:, self.handles["grip_site"], :]
        self._eef_lf_state = self._rigid_body_state[:, self.handles["leftfinger_tip"], :]
        self._eef_rf_state = self._rigid_body_state[:, self.handles["rightfinger_tip"], :]
        self._lf_force_state = self._net_cf[:, self.handles["leftfinger_tip"], :]
        self._rf_force_state = self._net_cf[:, self.handles["rightfinger_tip"], :]
        self._contact_forces = self._net_cf[:, self.handles["cubeA_body_handle"], :]
        _jacobian = self.gym.acquire_jacobian_tensor(self.sim, "franka")
        jacobian = gymtorch.wrap_tensor(_jacobian)
        hand_joint_index = self.gym.get_actor_joint_dict(env_ptr, franka_handle)['panda_hand_joint']
        self._j_eef = jacobian[:, hand_joint_index, :, :7]
        _massmatrix = self.gym.acquire_mass_matrix_tensor(self.sim, "franka")
        mm = gymtorch.wrap_tensor(_massmatrix)
        self._mm = mm[:, :7, :7]
        self._cubeA_state = self._root_state[:, self._cubeA_id, :]

        # Initialize states
        self.states.update({
            "cubeA_size": torch.ones_like(self._eef_state[:, 0]) * self.cubeA_size,
        })

        # Initialize actions
        self._pos_control = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float, device=self.device)

        # Initialize control
        self._arm_control = self._pos_control[:, :7]
        self._gripper_control = self._pos_control[:, 7:9]

        # Initialize indices
        self._global_indices = torch.arange(self.num_envs * self.num_actor, dtype=torch.int32,
                                            device=self.device).view(self.num_envs, -1)

    def _update_states(self):
        self.states.update({
            # Franka
            "q": self._q[:, :],
            "q_gripper": self._q[:, -2:],
            "eef_pos": self._eef_state[:, :3],
            "eef_quat": self._eef_state[:, 3:7],
            "eef_vel": self._eef_state[:, 7:],
            "eef_lf_pos": self._eef_lf_state[:, :3],
            "eef_rf_pos": self._eef_rf_state[:, :3],
            "lf_force": self._lf_force_state[:, :],
            "rf_force": self._rf_force_state[:, :],
            # Cubes
            "cubeA_quat": self._cubeA_state[:, 3:7],
            "cubeA_pos": self._cubeA_state[:, :3],
            "cubeA_to_eef_pos": self._cubeA_state[:, :3] - self._eef_state[:, :3],
            "cubeA_force": self._contact_forces[:, :],
        })

    def _refresh(self):
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.gym.refresh_jacobian_tensors(self.sim)
        self.gym.refresh_mass_matrix_tensors(self.sim)
        self.gym.refresh_net_contact_force_tensor(self.sim)

        # Refresh states
        self._update_states()

    def _reset_init_cube_state(self, env_ids, check_valid=True):
        """
        Simple method to sample @cube's position based on self.startPositionNoise and self.startRotationNoise, and automaticlly reset the pose internally. Populates the appropriate self._init_cubeX_state
        If @check_valid is True, then this will also make sure that the sampled position is not in contact with the  other cube.
        Args:
            env_ids (tensor or None): Specific environments to reset cube for
            check_valid (bool): Whether to make sure sampled position is collision-free with the other cube.
        """
        # If env_ids is None, we reset all the envs
        if env_ids is None:
            env_ids = torch.arange(start=0, end=self.num_envs, device=self.device, dtype=torch.long)

        # Initialize buffer to hold sampled values
        num_resets = len(env_ids)
        sampled_cube_state = torch.zeros(num_resets, 13, device=self.device)

        # Get correct references depending on which one was selected
        this_cube_state_all = self._init_cubeA_state
        cube_heights = self.states["cubeA_size"]

        # Minimum cube distance for guarenteed collision-free sampling is the sum of each cube's effective radius
        min_dists = (self.states["cubeA_size"])[env_ids] * np.sqrt(2) / 2.0

        # We scale the min dist by 2 so that the cubes aren't too close together
        min_dists = min_dists * 2.0

        # Sampling is "centered" around middle of table
        centered_cube_xy_state = torch.tensor(self._table_surface_pos[:2], device=self.device, dtype=torch.float32)

        # Set z value, which is fixed height
        sampled_cube_state[:, 2] = self._table_surface_pos[2] + cube_heights.squeeze(-1)[env_ids] / 2

        # Initialize rotation, which is no rotation (quat w = 1)
        sampled_cube_state[:, 6] = 1.0

        # If we're verifying valid sampling, we need to check and re-sample if any are not collision-free
        # We use a simple heuristic of checking based on cubes' radius to determine if a collision would occur
        if check_valid:
            success = False
            # Indexes corresponding to envs we're still actively sampling for
            active_idx = torch.arange(num_resets, device=self.device)
            num_active_idx = len(active_idx)
            for i in range(100):
                # Sample x y values
                sampled_cube_state[active_idx, :2] = centered_cube_xy_state + \
                                                     2.0 * self.start_position_noise * (torch.rand_like(sampled_cube_state[active_idx, :2]) - 0.5)
                # Check if sampled values are valid
                cube_dist = torch.linalg.norm(sampled_cube_state[:, :2], dim=-1)
                active_idx = torch.nonzero(cube_dist < min_dists, as_tuple=True)[0]
                num_active_idx = len(active_idx)
                # If active idx is empty, then all sampling is valid :D
                if num_active_idx == 0:
                    success = True
                    break
            # Make sure we succeeded at sampling
            assert success, "Sampling cube locations was unsuccessful! ):"
        else:
            # We just directly sample
            sampled_cube_state[:, :2] = centered_cube_xy_state.unsqueeze(0) + \
                                              2.0 * self.start_position_noise * (torch.rand(num_resets, 2, device=self.device) - 0.5)
            sampled_cube_state[:, 0] = -0.125

        # Sample rotation value
        if self.start_rotation_noise > 0:
            aa_rot = torch.zeros(num_resets, 3, device=self.device)
            aa_rot[:, 2] = 2.0 * self.start_rotation_noise * (torch.rand(num_resets, device=self.device) - 0.5)
            sampled_cube_state[:, 3:7] = quat_mul(axisangle2quat(aa_rot), sampled_cube_state[:, 3:7])

        # Lastly, set these sampled values as the new init state
        this_cube_state_all[env_ids, :] = sampled_cube_state

    def create_viewer(self):
        # create viewer for debugging (looking at the center of environment)
        viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        cam_pos = gymapi.Vec3(10, 0.0, 5)
        cam_target = gymapi.Vec3(-1, 0, 0)
        self.gym.viewer_camera_look_at(viewer, self.envs[self.args.num_envs // 2], cam_pos, cam_target)
        return viewer

    def get_obs(self, env_ids=None):
        # get state observation from each environment id
        if env_ids is None:
            env_ids = torch.arange(self.args.num_envs, device=self.args.sim_device)

        self._refresh()
        obs = ["cubeA_quat", "cubeA_pos", "cubeA_to_eef_pos", "eef_pos", "eef_quat", "q"]
        self.obs_buf = torch.cat([self.states[ob] for ob in obs], dim=-1)

        maxs = {ob: torch.max(self.states[ob]).item() for ob in obs}
        return self.obs_buf

    def get_reward(self):
        self.reward_buf[:], self.reset_buf[:] = compute_franka_reward(
            self.reset_buf, self.progress_buf, self.actions, self.states, self.reward_settings, self.max_episode_length
        )

    def reset(self):
        env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)

        if len(env_ids) == 0:
            return

        env_ids_int32 = env_ids.to(dtype=torch.int32)

        # Reset cubes
        self._reset_init_cube_state(env_ids=env_ids, check_valid=False)

        # Write these new init states to the sim states
        self._cubeA_state[env_ids] = self._init_cubeA_state[env_ids]

        # Reset agent
        reset_noise = torch.rand((len(env_ids), 9), device=self.device)
        pos = tensor_clamp(
            self.franka_default_dof_pos.unsqueeze(0) +
            self.franka_dof_noise * 2.0 * (reset_noise - 0.5),
            self.franka_dof_lower_limits.unsqueeze(0), self.franka_dof_upper_limits)

        # Overwrite gripper init pos (no noise since these are always position controlled)
        pos[:, -2:] = self.franka_default_dof_pos[-2:]

        # Reset the internal obs accordingly
        self._q[env_ids, :] = pos
        self._qd[env_ids, :] = torch.zeros_like(self._qd[env_ids])

        # Set any position control to the current position, and any vel / effort control to be 0
        # NOTE: Task takes care of actually propagating these controls in sim using the SimActions API
        self._pos_control[env_ids, :] = pos

        # Deploy updates
        multi_env_ids_int32 = self._global_indices[env_ids, 0].flatten()
        self.gym.set_dof_position_target_tensor_indexed(self.sim,
                                                        gymtorch.unwrap_tensor(self._pos_control),
                                                        gymtorch.unwrap_tensor(multi_env_ids_int32),
                                                        len(multi_env_ids_int32))
        
        self.gym.set_dof_state_tensor_indexed(self.sim,
                                            gymtorch.unwrap_tensor(self._dof_state),
                                            gymtorch.unwrap_tensor(multi_env_ids_int32),
                                            len(multi_env_ids_int32))

        # Update cube states
        multi_env_ids_cubes_int32 = self._global_indices[env_ids, -2:].flatten()
        self.gym.set_actor_root_state_tensor_indexed(
            self.sim, gymtorch.unwrap_tensor(self._root_state),
            gymtorch.unwrap_tensor(multi_env_ids_cubes_int32), len(multi_env_ids_cubes_int32))

        self.progress_buf[env_ids] = 0
        self.reset_buf[env_ids] = 0
        # refresh new observation after reset
        self.get_obs()

    def simulate(self):
        # step the physics
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

    def render(self):
        # update viewer
        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self.viewer, self.sim, True)
        self.gym.sync_frame_time(self.sim)

    def exit(self):
        # close the simulator in a graceful way
        if not self.args.headless:
            self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)

    def step(self, actions=None):
        if actions is not None:
            self.actions = actions.clone().to(self.device)

            targets = self._pos_control[:, :self.num_franka_dofs] + self.franka_dof_speed_scales * self.dt * self.actions * self.action_scale
            self._pos_control[:, :self.num_franka_dofs] = tensor_clamp(
                targets, self.franka_dof_lower_limits, self.franka_dof_upper_limits)
            env_ids_int32 = torch.arange(self.num_envs, dtype=torch.int32, device=self.device)

            # Deploy actions
            self.gym.set_dof_position_target_tensor(self.sim,
                                                    gymtorch.unwrap_tensor(self._pos_control))

        # simulate and render
        self.simulate()
        if not self.args.headless:
            self.render()

        # reset environments if required
        self.progress_buf += 1

        self.get_obs()
        self.get_reward()

        # TEST
        # Control gripper
        u_fingers = torch.zeros_like(self._gripper_control)
        u_fingers[:, 0] = 0.01
        u_fingers[:, 1] = 0.01
        self._gripper_control[:, :] = u_fingers
        if self.progress_buf[0] > 100:
            self.gym.set_dof_position_target_tensor(self.sim, gymtorch.unwrap_tensor(self._pos_control))

        # visualize vector
        self.gym.clear_lines(self.viewer)
        self.gym.refresh_force_sensor_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.gym.refresh_net_contact_force_tensor(self.sim)

        # print(self._sensor_state)
        print(self._contact_forces.shape)
        print(self._contact_forces)

        print(self._lf_force_state.shape)
        print(self._lf_force_state)

        print(self._rf_force_state.shape)
        print(self._rf_force_state)
 



# define reward function using JIT
@torch.jit.script
def compute_franka_reward(
    reset_buf, progress_buf, actions, states, reward_settings, max_episode_length
):
    # type: (Tensor, Tensor, Tensor, Dict[str, Tensor], Dict[str, float], float) -> Tuple[Tensor, Tensor]

    # Compute per-env physical parameters
    cubeA_size = states["cubeA_size"]

    # distance from hand to the cubeA
    d = torch.norm(states["cubeA_to_eef_pos"], dim=-1)
    d_lf = torch.norm(states["cubeA_pos"] - states["eef_lf_pos"], dim=-1)
    d_rf = torch.norm(states["cubeA_pos"] - states["eef_rf_pos"], dim=-1)
    dist_reward = 1 - torch.tanh(10.0 * (d + d_lf + d_rf) / 3)

    # reward for lifting cubeA
    cubeA_height = states["cubeA_pos"][:, 2] - reward_settings["table_height"]
    cubeA_lifted = (cubeA_height - cubeA_size) > 0.04
    lift_reward = cubeA_lifted

    # final reward for grasping successfully. TODO: replace force closure check
    gripper_grasping_cubeA = (d < 0.05)
    grasp_reward = gripper_grasping_cubeA & cubeA_lifted

    # We either provide the grasp reward or dist reward
    rewards = torch.where(
        grasp_reward,
        reward_settings["r_grasp_scale"] * grasp_reward,
        reward_settings["r_dist_scale"] * dist_reward + reward_settings["r_lift_scale"] * lift_reward,
    )
    # print("==========================================", progress_buf)
    # print("Grasp reward: ", reward_settings["r_grasp_scale"] * grasp_reward, d, d_lf, d_rf)
    # print("Dist reward: ", reward_settings["r_dist_scale"] * dist_reward)
    # print("Lift reward: ", reward_settings["r_lift_scale"] * lift_reward)
    # print("Rewards: ", rewards)

    # Compute resets
    reset_buf = torch.where((progress_buf >= max_episode_length - 1) | (grasp_reward > 0), torch.ones_like(reset_buf), reset_buf)

    return rewards, reset_buf