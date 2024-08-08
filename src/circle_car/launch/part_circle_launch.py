from launch import LaunchDescription
import launch_ros.actions
import launch.actions
import os
import sys

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='circle_car', executable='part_circle_controller', output='screen' ,)]
    )