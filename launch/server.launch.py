from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    # Andino launchfile
    package_name_andino = 'andino_gz_classic'
    package_dir_andino = get_package_share_directory(package_name_andino)
    andino_launch_file = os.path.join(
        package_dir_andino,  # Cambia este path al correcto
        'launch',
        'andino_one_robot.launch.py'
    )

    # Rosbridge server launchfile
    package_name_rosbridge_server = 'rosbridge_server'
    package_dir_rosbridge_server = get_package_share_directory(package_name_rosbridge_server)
    rosbridge_server_file = os.path.join(
        package_dir_rosbridge_server,  # Cambia este path al correcto
        'launch',
        'rosbridge_websocket_launch.xml'
    )


    # Configura los argumentos que quieres pasar
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='3.0')

    rosbrigde_server = ExecuteProcess(
    cmd=['ros2', 'launch', 'rosbridge_server', "rosbridge_websocket_launch.xml"],
    output='screen'
    )

    web_video_server = ExecuteProcess(
    cmd=['ros2', 'run', 'web_video_server', "web_video_server"],
    output='screen'
    )


    return LaunchDescription([
        # Nodo del servidor Flask
        Node(
            package='robot_manager',
            executable='server.py',
            name='robot_server',
            output='screen'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(andino_launch_file),
            launch_arguments={'initial_pose_x': initial_pose_x}.items(),
        ),

        rosbrigde_server,
        web_video_server,
    ])
