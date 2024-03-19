import os
import yourdfpy
import numpy as np
import time
from functools import partial
import argparse

def generate_joint_limit_trajectory(urdf_model, loop_time):
    """Generate a trajectory for all actuated joints that interpolates between joint limits.
    For continuous joint interpolate between [0, 2 * pi].

    Args:
        urdf_model (yourdfpy.URDF): _description_
        loop_time (float): Time in seconds to loop through the trajectory.

    Returns:
        dict: A dictionary over all actuated joints with list of configuration values.
    """
    trajectory_via_points = {}
    for joint_name in urdf_model.actuated_joint_names:
        if urdf_model.joint_map[joint_name].type.lower() == "continuous":
            via_point_0 = 0.0
            via_point_2 = 2.0 * np.pi
            via_point_1 = (via_point_2 - via_point_0) / 2.0
        else:
            limit_lower = (
                urdf_model.joint_map[joint_name].limit.lower
                if urdf_model.joint_map[joint_name].limit.lower is not None
                else -np.pi
            )
            limit_upper = (
                urdf_model.joint_map[joint_name].limit.upper
                if urdf_model.joint_map[joint_name].limit.upper is not None
                else +np.pi
            )
            via_point_0 = limit_lower
            via_point_1 = limit_upper
            via_point_2 = limit_lower

        trajectory_via_points[joint_name] = np.array(
            [
                via_point_0,
                via_point_1,
                via_point_2,
            ]
        )
    times = np.linspace(0.0, 1.0, int(loop_time * 100.0))
    bins = np.arange(3) / 2.0

    # Compute alphas for each time
    inds = np.digitize(times, bins, right=True)
    inds[inds == 0] = 1
    alphas = (bins[inds] - times) / (bins[inds] - bins[inds - 1])

    # Create the new interpolated trajectory
    trajectory = {}
    for k in trajectory_via_points:
        trajectory[k] = (
            alphas * trajectory_via_points[k][inds - 1]
            + (1.0 - alphas) * trajectory_via_points[k][inds]
        )

    return trajectory

def viewer_callback(scene, urdf_model, trajectory, loop_time):
    frame = int(100.0 * (time.time() % loop_time))
    cfg = {k: trajectory[k][frame] for k in trajectory}

    urdf_model.update_cfg(configuration=cfg)


if __name__ == "__main__":

    # get args from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', type=str, default=None)
    parser.add_argument("--animate", default=False, action="store_true",
                    help="Flag to do animation")
    args = parser.parse_args()
    urdf_path = args.urdf
    animate = args.animate

    if not urdf_path:
        file_path = os.path.abspath(__file__)
        urdf_path = os.path.join(os.path.dirname(file_path), '../../urdf/franka_description/robots/franka_panda.urdf')

    # load urdf
    urdf_model  = yourdfpy.URDF.load(urdf_path,
                                    build_collision_scene_graph=True,
                                    load_collision_meshes=True)

    callback = None
    if animate:
        loop_time = 6.0
        callback = partial(
            viewer_callback,
            urdf_model=urdf_model,
            loop_time=loop_time,
            trajectory=generate_joint_limit_trajectory(
                urdf_model=urdf_model, loop_time=loop_time
            ),
        )

    # print urdf info
    print(urdf_model.actuated_joint_names)
    print(list(urdf_model.link_map.keys()))

    # visualize urdf
    urdf_model.show(collision_geometry=False,
                    callback=callback)