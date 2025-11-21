from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('robotdog_description')

    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'robotdog.xacro'
    ])
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    controller_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'robotdog_controller_rviz. yaml'
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }]
    )

    # Controller Manager Node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_params_file
        ],
        output='screen',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description')
        ]
    )

    # Delay controller spawners
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    joint_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gazebo_joint_controller'],
                output='screen'
            )
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        joint_controller_spawner,
        rviz
    ])