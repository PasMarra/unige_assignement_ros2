from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='assignement',
            executable='controller',
            name='ctrl',
            namespace='assignement'
        ),
    ])