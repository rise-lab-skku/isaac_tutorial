import os
import math
from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(description="Joint control Methods Example")

# create a simulator
sim_params = gymapi.SimParams()
sim_params.substeps = 2
sim_params.dt = 1.0 / 60.0

sim_params.physx.solver_type = 1
sim_params.physx.num_position_iterations = 4
sim_params.physx.num_velocity_iterations = 1

sim_params.physx.num_threads = args.num_threads
sim_params.physx.use_gpu = args.use_gpu

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)

if sim is None:
    print("*** Failed to create sim")
    quit()

# create viewer using the default camera properties
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise ValueError('*** Failed to create viewer')

# add ground plane
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, gymapi.PlaneParams())

# set up the env grid
num_envs = 4
spacing = 1.5
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, 0.0, spacing)

# add cartpole urdf asset
asset_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../urdf')
asset_file = "cartpole.urdf"

# Load asset with default control type of position for all joints
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
cartpole_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# initial root pose for cartpole actors
initial_pose = gymapi.Transform()
initial_pose.p = gymapi.Vec3(0.0, 2.0, 0.0)
initial_pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)

# Create environment 0
# Cart held steady using position target mode.
# Pole held at a 45 degree angle using position target mode.
env0 = gym.create_env(sim, env_lower, env_upper, 2)
cartpole0 = gym.create_actor(env0, cartpole_asset, initial_pose, 'cartpole', 0, 1)
# Configure DOF properties
props = gym.get_actor_dof_properties(env0, cartpole0)
props["driveMode"] = (gymapi.DOF_MODE_POS, gymapi.DOF_MODE_POS)
props["stiffness"] = (5000.0, 5000.0)
props["damping"] = (100.0, 100.0)
gym.set_actor_dof_properties(env0, cartpole0, props)
# Set DOF drive targets
cart_dof_handle0 = gym.find_actor_dof_handle(env0, cartpole0, 'slider_to_cart')
pole_dof_handle0 = gym.find_actor_dof_handle(env0, cartpole0, 'cart_to_pole')
gym.set_dof_target_position(env0, cart_dof_handle0, 0)
gym.set_dof_target_position(env0, pole_dof_handle0, 0.25 * math.pi)

# Create environment 1
# Cart held steady using position target mode.
# Pole rotating using velocity target mode.
env1 = gym.create_env(sim, env_lower, env_upper, 2)
cartpole1 = gym.create_actor(env1, cartpole_asset, initial_pose, 'cartpole', 1, 1)
# Configure DOF properties
props = gym.get_actor_dof_properties(env1, cartpole1)
props["driveMode"] = (gymapi.DOF_MODE_POS, gymapi.DOF_MODE_VEL)
props["stiffness"] = (5000.0, 0.0)
props["damping"] = (100.0, 200.0)
gym.set_actor_dof_properties(env1, cartpole1, props)
# Set DOF drive targets
cart_dof_handle1 = gym.find_actor_dof_handle(env1, cartpole1, 'slider_to_cart')
pole_dof_handle1 = gym.find_actor_dof_handle(env1, cartpole1, 'cart_to_pole')
gym.set_dof_target_position(env1, cart_dof_handle1, 0)
gym.set_dof_target_velocity(env1, pole_dof_handle1, -2.0 * math.pi)

# Create environment 2
# Cart moving side to side using velocity target mode.
# Pole held steady using position target mode.
env2 = gym.create_env(sim, env_lower, env_upper, 2)
cartpole2 = gym.create_actor(env2, cartpole_asset, initial_pose, 'cartpole', 2, 1)
# Configure DOF properties
props = gym.get_actor_dof_properties(env2, cartpole2)
props["driveMode"] = (gymapi.DOF_MODE_VEL, gymapi.DOF_MODE_POS)
props["stiffness"] = (0.0, 5000.0)
props["damping"] = (200.0, 100.0)
gym.set_actor_dof_properties(env2, cartpole2, props)
# Set DOF drive targets
cart_dof_handle2 = gym.find_actor_dof_handle(env2, cartpole2, 'slider_to_cart')
pole_dof_handle2 = gym.find_actor_dof_handle(env2, cartpole2, 'cart_to_pole')
gym.set_dof_target_velocity(env2, cart_dof_handle2, 1.0)
gym.set_dof_target_position(env2, pole_dof_handle2, 0.0)

# Create environment 3
# Cart has no drive mode, but will be pushed around using forces.
# Pole held steady using position target mode.
env3 = gym.create_env(sim, env_lower, env_upper, 2)
cartpole3 = gym.create_actor(env3, cartpole_asset, initial_pose, 'cartpole', 3, 1)
# Configure DOF properties
props = gym.get_actor_dof_properties(env3, cartpole3)
props["driveMode"] = (gymapi.DOF_MODE_POS, gymapi.DOF_MODE_EFFORT)
props["stiffness"] = (5000.0, 0.0)
props["damping"] = (100.0, 0.0)
gym.set_actor_dof_properties(env3, cartpole3, props)
# Set DOF drive targets
cart_dof_handle3 = gym.find_actor_dof_handle(env3, cartpole3, 'slider_to_cart')
pole_dof_handle3 = gym.find_actor_dof_handle(env3, cartpole3, 'cart_to_pole')
gym.set_dof_target_position(env3, cart_dof_handle3, 0.0)
gym.apply_dof_effort(env3, pole_dof_handle3, 200)

# Look at the first env
cam_pos = gymapi.Vec3(8, 4, 1.5)
cam_target = gymapi.Vec3(0, 2, 1.5)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# Tensor API commanders
gym.prepare_sim(sim)
_rb_tensor = gym.acquire_rigid_body_state_tensor(sim)   # Get the info from envs
rb_tensor = gymtorch.wrap_tensor(_rb_tensor)            # Wrap the tensor
rb_positions = rb_tensor[:, 0:3]                        # Get the positions
rb_orientations = rb_tensor[:, 3:7]                     # Get the orientations
rb_linvels = rb_tensor[:, 7:10]                         # Get the linear velocities
rb_angvels = rb_tensor[:, 10:13]                        # Get the angular velocities

rb_names = gym.get_actor_rigid_body_names(env0, cartpole0) # Get the names of the rigid bodies

# Simulate
while not gym.query_viewer_has_closed(viewer):

    print(f"--------------------------------\n\
        Names:    :    \n{rb_names}\n\
        Positions :    \n{rb_positions}\n\
        Orientations :    \n{rb_orientations}\n\
        Linear Velocities :    \n{rb_linvels}\n\
        Angular Velocities :    \n{rb_angvels}\n")


    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)
    gym.refresh_rigid_body_state_tensor(sim) # Refresh the tensor

    # Nothing to be done for env 0

    # Nothing to be done for env 1

    # Update env 2: reverse cart target velocity when bounds reached
    pos = gym.get_dof_position(env2, cart_dof_handle2)
    if pos >= 0.5:
        gym.set_dof_target_velocity(env2, cart_dof_handle2, -1.0)
    elif pos <= -0.5:
        gym.set_dof_target_velocity(env2, cart_dof_handle2, 1.0)

    # Update env 3: apply an effort to the pole to keep it upright
    pos = gym.get_dof_position(env3, pole_dof_handle3)
    gym.apply_dof_effort(env3, pole_dof_handle3, -pos * 50)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

print('Done')

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
