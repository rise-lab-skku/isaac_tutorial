import argparse
from omni.isaac.lab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
"""Rest everything follows."""

import os
import omni.isaac.lab.sim as sim_utils
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg
from omni.isaac.lab.sim.converters import MeshConverter, MeshConverterCfg
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
from omni.isaac.lab.assets import Articulation, ArticulationCfg

ISAAC_TUTORIAL_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../")
FRANKA_PANDA_URDF = os.path.join(ISAAC_TUTORIAL_DIR, "urdf/franka_description/robots/franka_panda.urdf")

def design_scene():
    """Designs the scene by spawning ground plane, light, objects and meshes from usd files."""
    # Ground-plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # spawn distant light
    cfg_light = sim_utils.DomeLightCfg(
        intensity=1000.0,
        color=(1.0, 1.0, 1.0),
        exposure=1.5,
    )
    cfg_light.func("/World/DomeLight", cfg_light, translation=(0, 0, 2))

    # Create URDF converter config
    urdf_converter_cfg = UrdfConverterCfg(
        asset_path=FRANKA_PANDA_URDF,
        usd_dir=os.path.join(ISAAC_TUTORIAL_DIR, "usd/franka"),
        usd_file_name="franka_panda.usd",
        fix_base=True,
        self_collision=False,
        make_instanceable=True
    )
    # Create USD file using UrdfConverter
    urdf_converter = UrdfConverter(urdf_converter_cfg)

    # create a new xform prim for all objects to be spawned under
    prim_utils.create_prim("/World/Objects", "Xform")

    # rigid body properties
    rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True)

    # mass properties
    mass_props = sim_utils.MassPropertiesCfg(mass=1.0)

    # collision properties
    collision_props = sim_utils.CollisionPropertiesCfg(collision_enabled=True)

    # articulation properties
    articulation_props = sim_utils.ArticulationRootPropertiesCfg(articulation_enabled=True,
                                                                fix_root_link=True)

    # spawn a usd file of a table into the scene
    cfg_usd = sim_utils.UsdFileCfg(usd_path=os.path.join(ISAAC_TUTORIAL_DIR,
                                                        urdf_converter.usd_dir,
                                                        urdf_converter.usd_file_name),
                                    articulation_props=articulation_props,
                                    rigid_props=rigid_props,
    )
    cfg_usd.func("/World/Objects/Franka", cfg_usd, translation=(0.0, 0.0, 0.0))

    # Create Mesh converter config
    mesh_converter_cfg = MeshConverterCfg(
        asset_path=os.path.join(ISAAC_TUTORIAL_DIR, "mesh/duck.stl"),
        force_usd_conversion=True,
        usd_dir=os.path.join(ISAAC_TUTORIAL_DIR, "usd/duck"),
        usd_file_name="duck.usd",
        make_instanceable=False,
        collision_approximation="convexHull", # ["convexDecomposition", "convexHull", "none"]
        mass_props=mass_props,
        rigid_props=rigid_props,
        collision_props=collision_props)

    # Create USD file using MeshConverter
    mesh_converter = MeshConverter(mesh_converter_cfg)


    # spawn a usd file of a table into the scene
    cfg_usd = sim_utils.UsdFileCfg(usd_path=os.path.join(ISAAC_TUTORIAL_DIR,
                                                        mesh_converter.usd_dir,
                                                        mesh_converter.usd_file_name),
                                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0)),
    )
    cfg_usd.func("/World/Objects/Duck", cfg_usd, translation=(0.5, 0.0, 0.05))

def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # Design scene by adding assets to it
    design_scene()

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Simulate physics
    while simulation_app.is_running():
        # Perform step
        sim.step()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
