import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    stereo_odometry = get_package_share_directory("stereo_odometry")

    stereo_odometry_node =  Node(
            package='stereo_odometry',
            executable='stereo_odometry_node',
            name='stereo_odometry_node',
            parameters=[{
                "left_image_topic": "/left_image",
                "right_image_topic": "/right_image",
                "calibration_file": os.path.join(stereo_odometry, "config", "kitti.yaml"),
                  }]
        )
 
    ld = LaunchDescription()
    ld.add_action(stereo_odometry_node)

    return ld