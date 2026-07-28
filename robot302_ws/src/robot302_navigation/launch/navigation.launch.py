import os
from ament_index_python.packages import get_package_share_directory #[cite: 9]
from launch import LaunchDescription #[cite: 9]
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument #[cite: 9]
from launch.launch_description_sources import PythonLaunchDescriptionSource #[cite: 9]
from launch.substitutions import LaunchConfiguration #[cite: 9]

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup') #[cite: 9]
    pkg_robot_nav = get_package_share_directory('robot302_navigation') #[cite: 9]

    use_sim_time = LaunchConfiguration('use_sim_time') #[cite: 9]
    params_file = LaunchConfiguration('params_file') #[cite: 9]
    
    # Ambil argument 'map' yang dikirimkan oleh start_sim.py
    map_yaml_file_path = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'), #[cite: 9]
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_robot_nav, 'maps', 'my_map.yaml')), #[cite: 9]
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_robot_nav, 'config', 'nav2_params.yaml')), #[cite: 9]

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py') #[cite: 9]
            ),
            launch_arguments={
                'use_sim_time': use_sim_time, #[cite: 9]
                'map': map_yaml_file_path, 
                'params_file': params_file, #[cite: 9]
                'autostart': 'true' #[cite: 9]
            }.items()
        )
    ])