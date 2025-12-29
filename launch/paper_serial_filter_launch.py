#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'protobot'

    return LaunchDescription([
        Node(
            package=package_name,
            executable='paper_serial_filter.py',
            name='paper_serial_filter',
            output='screen',
            emulate_tty=True,  # supaya log nampak cantik

        )
    ])