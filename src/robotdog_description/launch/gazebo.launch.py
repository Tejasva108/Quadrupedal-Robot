from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('robotdog_description')

    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'robotdog_gazebo.xacro'
    ])
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    controller_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'robotdog_controller.yaml'
    ])

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Spawn the robot into Gazebo
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'robotdog',
                    '-topic', '/robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.8',
                ],
                output='screen'
            )
        ]
    )

    # Load joint state broadcaster
    load_joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                     'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    # Load joint trajectory controller
    load_joint_controller = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                     'gazebo_joint_controller'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_joint_controller
    ])