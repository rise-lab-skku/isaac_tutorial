export isaac_sim_package_path=$HOME/.local/share/ov/pkg/isaac-sim-4.1.0

export ROS_DISTRO=foxy

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again potentially leading to conflicts
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/omni.isaac.ros2_bridge/foxy/lib

# Run Isaac Sim command
$isaac_sim_package_path/isaac-sim.sh

source build_ws/foxy/foxy_ws/install/setup.bash
source build_ws/foxy/isaac_sim_ros_ws/install/local_setup.bash
