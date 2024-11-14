import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros.parameter_descriptions
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    world_name = os.getenv("WORLD_NAME")
    pkg_dir = get_package_share_directory('ponics')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # # Load the SDF file from "description" package
    # sdf_file  =  os.path.join(pkg_project_description, 'models', 'swift_pico', 'swift_pico.sdf')
    # with open(sdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_dir,
            'world',
            f'{world_name}.sdf -r' 
        ])}.items(),
    )

    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': os.path.join(pkg_project_bringup, 'config', 'swift_pico_bridge.yaml'),
    #     }],
    #     output='screen'
    # )

    ld = LaunchDescription()

    ld.add_action(gz_sim)

    return ld