import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ESP_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
    LIDAR_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

    pkg_description = get_package_share_directory('robot302_description')
    pkg_navigation = get_package_share_directory('robot302_navigation')
    pkg_lidar = get_package_share_directory('rplidar_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    xacro_file = os.path.join(pkg_description, 'urdf', 'robot302.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', ESP_PORT, '-b', '115200'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    odom_node = Node(
        package='robot_odometry',
        executable='odom_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    ekf_config = os.path.join(pkg_navigation, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': LIDAR_PORT,
            'serial_baudrate': '115200'
        }.items()
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mode': 'mapping',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'transform_publish_period': 0.05,
            'map_update_interval': 2.0,
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.1,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000
        }]
    )

    rviz_config = os.path.join(
        pkg_description,
        'rviz',
        'nav2_302sim_view.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),
        micro_ros_agent,
        robot_state_publisher,
        odom_node,
        ekf_node,
        lidar,
        slam_toolbox,
        rviz
    ])