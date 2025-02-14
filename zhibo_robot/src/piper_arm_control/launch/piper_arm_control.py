from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piper_arm_control',
            executable='sim_move_demo',
            name='sim_move_demo',
        )])
