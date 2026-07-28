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
    pkg_nav = get_package_share_directory('robot302_navigation')
    
    # UBAH: Arahkan ke package rplidar_ros sesuai dengan hardware kamu
    pkg_lidar = get_package_share_directory('rplidar_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    launch_nav = LaunchConfiguration('launch_nav', default='true')
    map_file = LaunchConfiguration('map')

    xacro_file = os.path.join(pkg_description, 'urdf', 'robot302.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    odom = Node(
        package='robot_odometry',
        executable='odom_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # =========================================================
    # TAMBAHAN BARU: Node EKF (robot_localization)
    # Memanggil file ekf.yaml yang berada di robot302_navigation
    # =========================================================
    ekf_config_path = os.path.join(pkg_nav, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # UBAH: Konfigurasi LiDAR menggunakan rplidar_a1_launch.py dan tambahan baudrate
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': '115200'
        }.items()
    )

    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items(),
        condition=IfCondition(launch_nav)
    )

    rviz_config = os.path.join(pkg_description, 'rviz', 'nav2_302sim_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('launch_nav', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_nav, 'maps', 'my_map.yaml')),

        micro_ros_agent,
        rsp,
        odom,
        ekf_node,           # <--- Node EKF ditambahkan ke urutan eksekusi
        lidar,
        launch_navigation,
        rviz,
    ])