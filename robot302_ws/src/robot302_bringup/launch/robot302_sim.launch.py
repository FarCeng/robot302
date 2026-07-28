import os
from ament_index_python.packages import get_package_share_directory #
from launch import LaunchDescription #[cite: 10]
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument #[cite: 10]
from launch.launch_description_sources import PythonLaunchDescriptionSource #[cite: 10]
from launch.substitutions import LaunchConfiguration #[cite: 10]
from launch_ros.actions import Node #[cite: 10]

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('robot302_gazebo') #[cite: 10]
    pkg_navigation = get_package_share_directory('robot302_navigation') #[cite: 10]
    
    # UBAH: Arahkan ke package deskripsi robot kamu sendiri, bukan nav2_bringup
    pkg_description = get_package_share_directory('robot302_description') 
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') #[cite: 10]
    headless = LaunchConfiguration('headless', default='false') #[cite: 10]
    
    map_file = LaunchConfiguration('map') #[cite: 10]
    world_file = LaunchConfiguration('world') #[cite: 10]
    rviz_config = LaunchConfiguration('rviz_config') #[cite: 10]

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py') #[cite: 10]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, #[cite: 10]
            'headless': headless, #[cite: 10]
            'world': world_file #[cite: 10]
        }.items()
    )

    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation.launch.py') #[cite: 10]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, #[cite: 10]
            'map': map_file #[cite: 10]
        }.items()
    )

    start_rviz2 = Node(
        package='rviz2', #[cite: 10]
        executable='rviz2', #[cite: 10]
        name='rviz2', #[cite: 10]
        arguments=['-d', rviz_config], #[cite: 10]
        parameters=[{'use_sim_time': use_sim_time}], #[cite: 10]
        output='screen' #[cite: 10]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Gunakan simulasi clock jika true'), #[cite: 10]
        DeclareLaunchArgument('headless', default_value='false', description='Jalankan Gazebo tanpa GUI jika true'), #[cite: 10]
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_navigation, 'maps', 'my_map.yaml'), description='Full path ke file peta .yaml'), #[cite: 10]
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_gazebo, 'worlds', 'warehouse.world'), description='Full path ke file world .world'), #[cite: 10]
        
        # UBAH: Deklarasi Argumen RViz diarahkan ke file rviz buatanmu sendiri
        DeclareLaunchArgument(
            'rviz_config', 
            default_value=os.path.join(pkg_description, 'rviz', 'nav2_302sim_view.rviz'), 
            description='Full path ke file config RViz2'
        ),

        launch_gazebo, #[cite: 10]
        launch_navigation, #[cite: 10]
        start_rviz2 #[cite: 10]
    ])