import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros.parameter_descriptions
from launch_ros.actions import Node
import xacro
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_name = os.getenv("WORLD_NAME")
    pkg_gazebo_dir = get_package_share_directory('ponics')
    pkg_project_description = get_package_share_directory('amr_mani_description')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'amr_mani', 'amr_ust.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
        # print(f"{robot_desc}")


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

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': LaunchConfiguration('pause_sim'),
            'world': f'{world_name}.sdf'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )




    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)




    return ld