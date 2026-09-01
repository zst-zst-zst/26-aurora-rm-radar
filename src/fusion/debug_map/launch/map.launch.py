import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    input_is_self_frame_arg = DeclareLaunchArgument(
        'input_is_self_frame',
        default_value='false',
        description='Whether the input is in self-view frame'
    )

    self_color_override_arg = DeclareLaunchArgument(
        'self_color_override',
        default_value='-1',
        description='Force self color: -1 auto, 0 blue, 2 red'
    )

    launch_path = os.path.join(
        get_package_share_directory('debug_map'),
        'launch',
        'debug_map.launch.py'
    )

    debug_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'input_is_self_frame': LaunchConfiguration('input_is_self_frame'),
            'self_color_override': LaunchConfiguration('self_color_override'),
        }.items(),
    )

    return LaunchDescription([
        input_is_self_frame_arg,
        self_color_override_arg,
        debug_map_launch,
    ])
