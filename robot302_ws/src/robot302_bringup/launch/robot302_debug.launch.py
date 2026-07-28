import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('robot302_description')
    pkg_bringup = get_package_share_directory('robot302_bringup')
    pkg_nav = get_package_share_directory('robot302_navigation')
    pkg_lidar = get_package_share_directory('sllidar_ros2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    headless = LaunchConfiguration('headless', default='false')

    xacro_file = os.path.join(pkg_description, 'urdf', 'robot302.urdf.xacro')
    robot_description = ParameterValue(
    Command(
        ['xacro ', xacro_file]
    ),
    value_type=str
    )

    # micro-ROS agent
    # sesuaikan port kalau beda, misal /dev/ttyACM0 atau /dev/ttyUSB0
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        output='screen'
    )

    # robot_state_publisher: wajib supaya URDF hidup di TF tree
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # odometry node Anda
    odom = Node(
        package='robot_odometry',
        executable='odom_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # lidar real robot
    # ganti launch ini sesuai model lidar Anda
    lidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            pkg_lidar,
            'launch',
            'sllidar_a1_launch.py'
        )
    ),
    launch_arguments={
        'serial_port': '/dev/ttyUSB1'
    }.items()
    )

    # RViz
    rviz_config = os.path.join(pkg_description, 'rviz', 'robot302.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),

        micro_ros_agent,
        rsp,
        odom,
        lidar,
        rviz,
    ])