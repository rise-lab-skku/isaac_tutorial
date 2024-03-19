#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import os
import numpy as np
from omni.isaac.core import World
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.viewports import set_camera_view

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
set_camera_view(eye=[7.5, 7.5, 7.5],
                target=[0.0, 0.0, 0.0])

asset_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../usd')
asset_file = "test.usd"
usd_path = os.path.join(asset_root, asset_file)

myrobot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/simple_robot",
        name="simple_robot",
        wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
        create_robot=True,
        usd_path=usd_path,
        position=[0, 0.0, 0.5],
    )
)

my_controller = DifferentialController(name="simple_control", wheel_radius=0.5, wheel_base=3)
my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        if i >= 0 and i < 500:
            # forward
            myrobot.apply_wheel_actions(my_controller.forward(command=[0.1, 0]))
            print(myrobot.get_linear_velocity())
        elif i >= 500 and i < 800:
            # rotate
            myrobot.apply_wheel_actions(my_controller.forward(command=[0.0, np.pi / 6]))
            print(myrobot.get_angular_velocity())
        elif i >= 800 and i < 1000:
            # forward
            myrobot.apply_wheel_actions(my_controller.forward(command=[0.1, 0]))
        elif i == 1000:
            i = 0
        i += 1


simulation_app.close()
