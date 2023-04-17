from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.actions
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rpi2_delfin').find('rpi2_delfin')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        Node(
            package='rpi2_delfin',
            namespace='rpi2_delfin',
            executable='talker',
            name='talker',
        )
    ])
