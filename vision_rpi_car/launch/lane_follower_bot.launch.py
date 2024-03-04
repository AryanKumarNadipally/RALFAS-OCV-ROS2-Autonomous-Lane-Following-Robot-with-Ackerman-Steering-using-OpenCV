import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    os.system("export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")

    return LaunchDescription(
        [

            Node(
                package="vision_rpi_car",
                executable="image_publisher_node",
                name="RPi_image_publisher",
                output='screen'
            ),
            Node(
                package="vision_rpi_car",
                executable="lane_follow_node",
                name="lane_follower",
                output='screen'
            ),

        ]
    )


