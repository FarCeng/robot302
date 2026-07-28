import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def generate_launch_description():
    pkg_description = get_package_share_directory("robot302_description") #[cite: 8]
    pkg_gazebo = get_package_share_directory("robot302_gazebo") #[cite: 8]

    use_sim_time = LaunchConfiguration('use_sim_time') #[cite: 8]
    headless = LaunchConfiguration('headless') #[cite: 8]
    
    # Ambil argument 'world' yang dikirimkan oleh start_sim.py
    world_path = LaunchConfiguration('world')

    xacro_file = os.path.join(pkg_description, "urdf", "robot302.urdf.xacro") #[cite: 8]
    robot_description_raw = xacro.process_file(xacro_file).toxml() #[cite: 8]

    start_gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], #[cite: 8]
        output='screen' #[cite: 8]
    )

    start_gzclient = ExecuteProcess(
        cmd=['gzclient'], #[cite: 8]
        output='screen', #[cite: 8]
        condition=UnlessCondition(headless) #[cite: 8]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher", #[cite: 8]
        executable="robot_state_publisher", #[cite: 8]
        output="screen", #[cite: 8]
        parameters=[{"robot_description": robot_description_raw, "use_sim_time": use_sim_time}], #[cite: 8]
    )

    spawn_entity = Node(
        package="gazebo_ros", #[cite: 8]
        executable="spawn_entity.py", #[cite: 8]
        arguments=["-topic", "robot_description", "-entity", "robot302"], #[cite: 8]
        output="screen", #[cite: 8]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'), #[cite: 8]
        DeclareLaunchArgument('headless', default_value='false'), #[cite: 8]
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_gazebo, "worlds", "warehouse.world")),
        start_gzserver, #[cite: 8]
        start_gzclient, #[cite: 8]
        robot_state_publisher, #[cite: 8]
        spawn_entity, #[cite: 8]
    ])