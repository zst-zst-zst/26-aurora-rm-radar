import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_runtime_config():
    candidates = [
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), '../../../../config/radar_runtime.yaml')
        ),
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), '../../../../../config/radar_runtime.yaml')
        ),
    ]
    for config_path in candidates:
        if os.path.exists(config_path):
            with open(config_path, 'r', encoding='utf-8') as file:
                return yaml.safe_load(file) or {}
    raise FileNotFoundError('Cannot find config/radar_runtime.yaml from launch path')


def generate_launch_description():
    runtime_config = _load_runtime_config()
    map_config = runtime_config.get('map', {})
    debug_config = runtime_config.get('debug_map', {})
    pre_match_config = runtime_config.get('pre_match', {})
    pre_match_team = str(pre_match_config.get('team', '')).strip()
    _default_self_color = '2' if pre_match_team == '0' else '0' if pre_match_team == '1' else '-1'

    input_is_self_frame_arg = DeclareLaunchArgument(
        'input_is_self_frame',
        default_value='false',
        description='Whether the input is in self-view frame'
    )

    self_color_override_arg = DeclareLaunchArgument(
        'self_color_override',
        default_value=_default_self_color,
        description='Force self color: -1 auto, 0 blue, 2 red'
    )

    return LaunchDescription([
        input_is_self_frame_arg,
        self_color_override_arg,
        Node(
            package='debug_map',
            executable='debug_map',
            name='debug_map',
            output='screen',
            parameters=[{
                'map_image': map_config.get('map_image', 'config/map/map.jpg'),
                'map_width': float(map_config.get('width', 28.0)),
                'map_height': float(map_config.get('height', 15.0)),
                'hero_threshold': debug_config.get('hero_threshold', 8.668),
                'hero_mid_x_min': debug_config.get('hero_mid_x_min', 20.3),
                'hero_mid_x_max': debug_config.get('hero_mid_x_max', 25.075),
                'hero_red_y_min': debug_config.get('hero_red_y_min', 10.3),
                'hero_blue_y_max_offset': debug_config.get('hero_blue_y_max_offset', 10.3),
                'input_is_self_frame': ParameterValue(
                    LaunchConfiguration('input_is_self_frame'),
                    value_type=bool,
                ),
                'self_color_override': ParameterValue(
                    LaunchConfiguration('self_color_override'),
                    value_type=int,
                ),
            }],
        ),
    ])
