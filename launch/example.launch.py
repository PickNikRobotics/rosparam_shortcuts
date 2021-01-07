import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    example = Node(package='rosparam_shortcuts',
                   executable='example',
                   output='screen',
                   parameters=[
                       os.path.join(get_package_share_directory("rosparam_shortcuts"), "config", "example.yaml")])

    return LaunchDescription([example])
