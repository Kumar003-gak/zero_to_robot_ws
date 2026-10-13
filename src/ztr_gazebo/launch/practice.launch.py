import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    race = os.path.join(get_package_share_directory("ztr_gazebo"), "launch", "race.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("team", default_value="1", description="Your team number (1-6)"),
        DeclareLaunchArgument("max_speed", default_value="5.0",
                              description="Car speed limit, m/s"),
        DeclareLaunchArgument("topic_prefix", default_value="",
                              description="MQTT topic prefix of your ESP32 joystick (empty = keyboard)"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(race),
            launch_arguments={
                "teams": LaunchConfiguration("team"),
                "max_speed": LaunchConfiguration("max_speed"),
                "follow": "true",
                "topic_prefix": LaunchConfiguration("topic_prefix"),
            }.items(),
        ),
    ])
