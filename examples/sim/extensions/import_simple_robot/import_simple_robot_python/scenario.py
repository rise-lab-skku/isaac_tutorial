# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os
import numpy as np
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import distance_metrics
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_rot_matrices
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.robots import WheeledRobot

class SimpleRobotExampleScript:
    def __init__(self):
        self._robot = None
        self._robot_controller = DifferentialController(name="simple_control", wheel_radius=0.5, wheel_base=3)

    def load_example_assets(self):
        """Load assets onto the stage and return them so they can be registered with the
        core.World.

        This function is called from ui_builder._setup_scene()

        The position in which things are loaded is also the position to which
        they will be returned on reset.
        """

        robot_prim_path = "/simple_robot"
        usd_path = os.path.join(os.path.dirname(__file__), "../../../../../usd")
        path_to_robot_usd = usd_path + "/simple_robot.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._robot = WheeledRobot(
                        prim_path=robot_prim_path,
                        name="simple_robot",
                        wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
                        create_robot=True,
                        usd_path=path_to_robot_usd,
                        position=[0, 0.0, 0.5],
                    )

        self._ground_plane = GroundPlane("/World/Ground")

        # Return assets that were added to the stage so that they can be registered with the core.World
        return self._robot, self._ground_plane

    def setup(self):
        """
        This function is called after assets have been loaded from ui_builder._setup_scenario().
        """
        # Set a camera view that looks good
        set_camera_view(eye=[6.5, 6.5, 4.5], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        self.reset()

    def reset(self):
        """
        This function is called when the reset button is pressed.
        In this example the core.World takes care of all necessary resetting
        by putting everything back in the position it was in when loaded.

        In more complicated scripts, e.g. scripts that modify or create USD properties
        or attributes at runtime, the user will need to implement necessary resetting
        behavior to ensure their script runs deterministically.
        """
        pass

    """
    The following two functions demonstrate the mechanics of running code in a script-like way
    from a UI-based extension.  This takes advantage of Python's yield/generator framework.  

    The update() function is tied to a physics subscription, which means that it will be called
    one time on every physics step (usually 60 frames per second).  Each time it is called, it
    queries the script generator using next().  This makes the script generator execute until it hits
    a yield().  In this case, no value need be yielded.  This behavior can be nested into subroutines
    using the "yield from" keywords.
    """

    def update(self, step: float, lin_vel: float, ang_vel: float):
        self.move_robot(lin_vel, ang_vel)

        # ang_vel = self._robot.get_angular_velocity()
        # lin_vel = self._robot.get_linear_velocity()

        position, orientation = self._robot.get_local_pose()
        state = self._robot.get_default_state()
        left_joint_vel, right_joint_vel = self._robot.get_joint_velocities()

    ################################### Functions

    def move_robot(self, lin_vel=0.0, ang_vel=0.0):
        # Move the robot by setting the left and right wheel speeds.
        self._robot.apply_wheel_actions(self._robot_controller.forward(command=[lin_vel, ang_vel]))