#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 启动 realsense2_camera 节点
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth': True,
        }]
    )

    # 启动 circle_detector 节点
    detector_node = Node(
        package='circle_detector_ros2',
        executable='circle_detector_node',
        name='circle_detector',
        output='screen'
    )

    return LaunchDescription([
        realsense_node,
        detector_node,
    ])
