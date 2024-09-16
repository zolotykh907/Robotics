from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Запуск трех окон симулятора черепах
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle2'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle3'
        ),

        # Запуск узлов mimic для того, чтобы вторая черепаха следовала за первой
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic_turtle2',
            parameters=[
                {"input_topic": "/turtle1/cmd_vel"},  # Вторая черепаха копирует команды первой
                {"output_topic": "/turtle2/cmd_vel"}
            ]
        ),

        # Запуск узла mimic для того, чтобы третья черепаха следовала за второй
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic_turtle3',
            parameters=[
                {"input_topic": "/turtle2/cmd_vel"},  # Третья черепаха копирует команды второй
                {"output_topic": "/turtle3/cmd_vel"}
            ]
        )
    ])
