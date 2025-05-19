import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # same-workspace package
    sbg_launch = os.path.join(
        get_package_share_directory('mt10_control'),
        'launch', 'mt_sbg_launch.py'
    )

    witmoiton_launch = os.path.join(
        get_package_share_directory('witmotion_ros'),
        'launch', 'wt905.launch.py'
    )

    # external workspace package (must have been sourced)
    ublox_launch = os.path.join(
        get_package_share_directory('ublox_gps'),
        'launch', 'ublox_gps_node_zedf9p-launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sbg_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(witmoiton_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ublox_launch)
        ),
    ])
