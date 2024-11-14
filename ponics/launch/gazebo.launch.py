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
import time


def generate_launch_description():
    world_name = os.getenv("WORLD_NAME")
    pkg_gazebo_dir = get_package_share_directory('ponics')
    pkg_project_description = get_package_share_directory('amr_mani_description')

    xacro_file = os.path.join(pkg_project_description, 'urdf', 'amr_mani.xacro')
    robot_description_config = xacro.process_file(xacro_file) 
    robot_desc = robot_description_config.toxml()

    world = os.path.join(pkg_gazebo_dir, 'world', f'{world_name}.world')


    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use simulation time or not'
    )

    pause_sim_arg = DeclareLaunchArgument(
        'pause_sim',
        default_value='true',
        description='whether to pause to play the simulation'
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
            'world': world
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

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'amr_mani',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )

    joint_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    joint_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager']
    )

    simple_base_motion_node = Node(
        package='goal_pose_commander',
        executable='go_to_pose_node',
        name='go_to_pose',
        output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(pause_sim_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(urdf_spawn_node)
    # ld.add_action(joint_broadcaster_spawner)
    # ld.add_action(joint_trajectory_spawner)
    ld.add_action(simple_base_motion_node)

    return ld