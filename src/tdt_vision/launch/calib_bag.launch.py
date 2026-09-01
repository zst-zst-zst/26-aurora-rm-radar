import os
import sys

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(__file__))
from _common import (
    apply_team_override,
    effective_self_color,
    load_runtime_config,
    select_calibration_path,
    system_libusb_preload_action,
)


def generate_launch_description():
    runtime_config = load_runtime_config(__file__)
    pre_match_config = runtime_config.get('pre_match', {})
    bag_config = runtime_config.get('bag', {})
    map_config = runtime_config.get('map', {})
    calibration_config = runtime_config.get('calibration', {})
    runtime_overrides = runtime_config.get('runtime', {})

    commit_arg = DeclareLaunchArgument(
        'sandbox',
        default_value='false',
        description='sandbox=true: 标定结果写入临时沙盒文件而非生产文件'
    )
    # 默认直接写生产文件；加 sandbox:=true 才用沙盒
    _sandbox = any(a.lower() in ('sandbox:=true', 'sandbox:=1', 'sandbox:=yes')
                   for a in sys.argv)

    bag_team_raw = bag_config.get('team') if isinstance(bag_config, dict) else None
    if bag_team_raw is not None:
        calib_team_config = dict(pre_match_config)
        calib_team_config['team'] = bag_team_raw
    else:
        calib_team_config = pre_match_config
    runtime_for_calibration = apply_team_override(runtime_overrides, calib_team_config)

    selected_color = effective_self_color(calibration_config, runtime_for_calibration)
    calibrate_points_path = select_calibration_path(
        calibration_config,
        runtime_for_calibration,
        'calibrate_points',
        'config/calibrate_points_red.yaml',
    )
    # 回放标定和实时标定共用同一套相机内参，避免同一相机被两套 K/dist 分叉。
    camera_params_path = select_calibration_path(
        calibration_config,
        runtime_for_calibration,
        'camera_params',
        'src/tdt_vision/camera/config/hik.yaml',
    )
    for arg in sys.argv:
        if arg.startswith('camera_params:='):
            camera_params_path = arg.split(':=', 1)[1]
            break
    bag_section = bag_config if isinstance(bag_config, dict) else {}
    # 标定结果优先写到 bag 专用的 out_matrix (与比赛实时 out_matrix 隔离),
    # 否则才回退到全局 calibration.out_matrix_*.
    bag_out_key = 'out_matrix_red' if selected_color == 2 else \
                  'out_matrix_blue' if selected_color == 0 else None
    bag_out_override = bag_section.get(bag_out_key) if bag_out_key else None
    if not bag_out_override:
        bag_out_override = bag_section.get('out_matrix')
    if bag_out_override:
        real_out_matrix_path = str(bag_out_override)
    else:
        real_out_matrix_path = select_calibration_path(
            calibration_config,
            runtime_for_calibration,
            'out_matrix',
            'config/out_matrix.yaml',
        )
    if _sandbox:
        out_matrix_path = 'config/out_matrix.yaml'
        print(
            f"[calib_bag.launch] 沙盒模式: out_matrix={out_matrix_path}"
            f"  (生产文件: {real_out_matrix_path})"
        )
    else:
        out_matrix_path = real_out_matrix_path
        print(
            f"[calib_bag.launch] effective_self_color={selected_color}, "
            f"calibrate_points={calibrate_points_path}, "
            f"camera_params={camera_params_path}, "
            f"out_matrix={out_matrix_path}"
        )

    def get_rosbag_player_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='rosbag_player_node',
            parameters=[{
                'rosbag_file': bag_config.get('calib_rosbag_file',
                              bag_config.get('rosbag_file', 'bags/latest')),
                # 标定不需要 /match_info；老 bag 的 MatchInfo 二进制布局可能跟当前 .msg 不兼容。
                'publish_match_info': False,
                'publish_lidar': False,
                # 标定只需低帧率，限速防止 decode 队列暴涨丢帧
                'camera_max_fps': float(bag_config.get('calib_camera_max_fps', 5.0)),
                'use_bag_timing': False,
                'wait_on_lidar_timing': False,
                'legacy_mode': True,
                'legacy_cycle_ms': int(bag_config.get('calib_legacy_cycle_ms', 200)),
                'decode_compressed_image': bag_config.get('decode_compressed_image', True),
                }],
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
                'team_label': 'RED' if selected_color == 2 else ('BLUE' if selected_color == 0 else ''),
            }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(radar_calib_node, ros_bag_player_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                radar_calib_node,
                ros_bag_player_node
            ],
            output='both',
            emulate_tty=True,
        )

    radar_calib_node = get_radar_calib_node('tdt_vision', 'tdt_radar::Calibrate')
    ros_bag_player_node = get_rosbag_player_node('rosbag_player', 'RosbagPlayer')
    cam_detector = get_camera_detector_container(radar_calib_node, ros_bag_player_node)

    return LaunchDescription([
        commit_arg,
        system_libusb_preload_action(),
        cam_detector,
    ])
