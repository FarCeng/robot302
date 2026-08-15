import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import (
    Command,
    LaunchConfiguration
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# =========================================================
# PERSISTENT USB DEVICES
# =========================================================

ESP_PORT = (
    "/dev/serial/by-id/"
    "usb-1a86_USB_Serial-if00-port0"
)

LIDAR_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def generate_launch_description():

    # =====================================================
    # PACKAGES
    # =====================================================

    pkg_description = get_package_share_directory(
        "robot302_description"
    )

    pkg_lidar = get_package_share_directory(
        "sllidar_ros2"
    )

    # =====================================================
    # ARGUMENTS
    # =====================================================

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    use_rviz = LaunchConfiguration(
        "use_rviz"
    )

    use_lidar = LaunchConfiguration(
        "use_lidar"
    )

    # =====================================================
    # ROBOT DESCRIPTION
    # =====================================================

    xacro_file = os.path.join(
        pkg_description,
        "urdf",
        "robot302.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file
            ]
        ),
        value_type=str
    )

    # =====================================================
    # MICRO-ROS AGENT
    # =====================================================

    micro_ros_agent = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "micro_ros_agent",
            "micro_ros_agent",
            "serial",
            "--dev",
            ESP_PORT,
            "-b",
            "115200",
        ],
        output="screen",
    )

    # =====================================================
    # ROBOT STATE PUBLISHER
    #
    # Publishes:
    # base_footprint -> base_link
    # and other URDF transforms.
    # =====================================================

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_debug",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # =====================================================
    # DEBUG ODOMETRY
    #
    # IMPORTANT:
    # - Does NOT publish /odom
    # - Publishes /odom_debug
    # - Publishes TF:
    #
    #       odom_debug
    #           |
    #      base_footprint
    #
    # This prevents conflict with the normal EKF setup.
    # =====================================================

    odom_debug = Node(
        package="robot_odometry",
        executable="odom_debug_node",
        name="encoder_odometry_debug_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # =====================================================
    # LIDAR
    # =====================================================

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_lidar,
                "launch",
                "sllidar_a1_launch.py",
            )
        ),
        launch_arguments={
            "serial_port": LIDAR_PORT,
        }.items(),
        condition=IfCondition(use_lidar),
    )

    # =====================================================
    # RVIZ
    # =====================================================

    rviz_config = os.path.join(
        pkg_description,
        "rviz",
        "robot302.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_debug",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(use_rviz),
    )

    # =====================================================
    # LAUNCH DESCRIPTION
    # =====================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock"
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz"
        ),

        DeclareLaunchArgument(
            "use_lidar",
            default_value="false",
            description="Launch LiDAR during odometry debug"
        ),

        # -----------------------------
        # Hardware
        # -----------------------------

        micro_ros_agent,

        # -----------------------------
        # Robot TF
        # -----------------------------

        rsp,

        # -----------------------------
        # Debug Odometry
        # -----------------------------

        odom_debug,

        # -----------------------------
        # Optional LiDAR
        # -----------------------------

        lidar,

        # -----------------------------
        # RViz
        # -----------------------------

        rviz,
    ])