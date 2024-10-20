import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('learning_tf2_py'), 'rviz', 'carrot.rviz')
    
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_py'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
       launch_arguments={'target_frame': 'carrot1'}.items(),
       )

    return LaunchDescription([
        demo_nodes,
        Node(
            package='learning_tf2_py',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
            parameters=[
                            {'radius': 4},
                            {'direction_of_rotation': 1}
            ]
        ),
        Node(
                    package='turtlesim',
                    executable='turtle_teleop_key',
                    name='turtle_teleop_key',
                    prefix='xterm -e',
                ),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen'
                ),
    ])
