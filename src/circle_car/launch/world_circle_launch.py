from launch import LaunchDescription
import launch_ros.actions
import launch.actions
import os
import sys

def generate_launch_description():
    gazebo_world_path = os.path.join(os.getcwd(), 'src/circle_car/world')
    gazebo_world = os.path.join(gazebo_world_path, 'gazebo_diff_drive_circle_test.world')
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', gazebo_world],
            additional_env={'GAZEBO_MODEL_PATH': gazebo_world_path})])
