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
    pkg_gazebo_dir = get_package_share_directory('ponics')
    pkg_project_description = get_package_share_directory('amr_mani_description')
    bridge_params_fp = os.path.join(pkg_gazebo_dir, 'config', 'bridge_params.yaml')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'amr_mani', 'amr_ust.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()


    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use simulation time or not'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_desc
            }
        ]
    )

    gz_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_dir, 'launch', 'world.launch.py'))
    )

    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params_fp}',
        ],
        output='screen',
    ) 


    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(gz_launcher)
    ld.add_action(gz_ros2_bridge)


    return ld