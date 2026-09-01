import os
import sys

os.environ.setdefault('ALLUSERSPROFILE', os.path.abspath(os.path.join(os.getcwd(), 'GenICamCache')))

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(__file__))
from _common import (
    BRAND_CAMERA_PARAMS,
    apply_team_override,
    effective_brand,
    effective_self_color,
    load_runtime_config,
    resolve_workspace_config_dir,
    select_calibration_path,
    system_libusb_preload_action,
    warn_if_same_side_calibration,
)


def generate_launch_description():
    runtime_config = load_runtime_config(__file__)
    pre_match_config = runtime_config.get('pre_match', {})
    map_config = runtime_config.get('map', {})
    calibration_config = runtime_config.get('calibration', {})
    warn_if_same_side_calibration(calibration_config, 'calib.launch')

    runtime_for_calibration = apply_team_override(
        runtime_config.get('runtime', {}), pre_match_config)

    brand = effective_brand(pre_match_config)
    brand_params_path = BRAND_CAMERA_PARAMS.get(brand)
    if brand_params_path:
        calibration_config = dict(calibration_config)
        calibration_config['camera_params_red'] = brand_params_path
        calibration_config['camera_params_blue'] = brand_params_path

    selected_color = effective_self_color(calibration_config, runtime_for_calibration)
    calibrate_points_path = select_calibration_path(
        calibration_config, runtime_for_calibration,
        'calibrate_points', 'config/calibrate_points_red.yaml',
    )
    camera_params_path = select_calibration_path(
        calibration_config, runtime_for_calibration,
        'camera_params', 'config/camera_params.yaml',
    )
    out_matrix_path = select_calibration_path(
        calibration_config, runtime_for_calibration,
        'out_matrix', 'config/out_matrix.yaml',
    )
    print(
        f"[calib.launch] effective_self_color={selected_color}, "
        f"calibrate_points={calibrate_points_path}, "
        f"camera_params={camera_params_path}, "
        f"out_matrix={out_matrix_path}"
    )

    workspace_config_dir = resolve_workspace_config_dir(__file__)
    camera_profile = 'night'
    camera_config_file = os.path.join(workspace_config_dir, 'camera_driver_night.yaml')
    print(f"[calib.launch] team={'red' if selected_color==2 else 'blue'}, "
          f"brand={brand}, camera_profile={camera_profile}, "
          f"camera_params={camera_params_path}")

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='vision_camera_node',
            parameters=[
                camera_config_file,
                {'view_local': False},
                {'brand': brand},
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_radar_calib_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_calib_node',
            parameters=[{
                'calibrate_points': calibrate_points_path,
                'camera_params': camera_params_path,
                'out_matrix': out_matrix_path,
                'map_points': map_config.get('map_points', 'config/map/map_points.yaml'),
                'map_height': float(map_config.get('height', 15.0)),
            }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node, calib_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[camera_node, calib_node],
            output='both',
            emulate_tty=True,
        )

    calib_node = get_radar_calib_node('tdt_vision', 'tdt_radar::Calibrate')
    hik_camera_node = get_camera_node('tdt_vision', 'tdt_vision::NodeCamera')
    cam_detector = get_camera_detector_container(hik_camera_node, calib_node)

    return LaunchDescription([
        system_libusb_preload_action(),
        cam_detector,
    ])
