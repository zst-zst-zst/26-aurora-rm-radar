import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from _common import (
    camera_center_from_extrinsics as _camera_center_from_extrinsics,
    default_serial_port as _default_serial_port,
    effective_self_color as _effective_self_color,
    load_runtime_config,
    select_calibration_path as _select_calibration_path,
    system_libusb_preload_action as _system_libusb_preload_action,
    warn_if_same_side_calibration,
)


def _load_runtime_config():
    return load_runtime_config(__file__)


def _warn_if_same_side_calibration(calibration_config):
    warn_if_same_side_calibration(calibration_config, 'bag.launch')


def generate_launch_description():
    pkg_share_path = get_package_share_directory("tdt_vision")
    lidar_pkg_share_path = get_package_share_directory("dynamic_cloud")
    serial_pkg_share_path = get_package_share_directory("kalman_filter")
    runtime_config = _load_runtime_config()
    # bag 也强制清掉旧的 refined map: 不同 bag 可能是不同摆位录制的,
    # 复用上一次的 refined 会让 ICP 在新 bag 上 reject. 始终从 CAD map.pcd 起步.
    _bag_refined_pcd = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..', '..', 'config', 'map', 'map_refined.pcd'))
    if os.path.exists(_bag_refined_pcd):
        try:
            os.remove(_bag_refined_pcd)
            print(f"[bag.launch] Removed stale {_bag_refined_pcd}")
        except OSError as e:
            print(f"[bag.launch][WARN] Failed to remove {_bag_refined_pcd}: {e}")
    pre_match_config = runtime_config.get('pre_match', {})
    map_config = runtime_config.get('map', {})
    bag_config = runtime_config.get('bag', {})
    calibration_config = runtime_config.get('calibration', {})
    _warn_if_same_side_calibration(calibration_config)
    runtime_overrides = runtime_config.get('runtime', {})
    runtime_for_calibration = dict(runtime_overrides)
    # bag.team 优先于 pre_match.team，让回放和实时比赛各自独立配置队伍
    bag_team_raw = bag_config.get('team')
    if bag_team_raw is not None:
        bag_team = str(bag_team_raw).strip()
    else:
        bag_team = str(pre_match_config.get('team', '')).strip()
    if bag_team == '0':
        runtime_for_calibration['self_color_override'] = 2
    elif bag_team == '1':
        runtime_for_calibration['self_color_override'] = 0

    pre_match_self_frame = str(pre_match_config.get('self_frame', '1')).strip().lower()
    input_is_self_frame_default = (
        "true" if pre_match_self_frame in ("1", "true", "yes", "on") else "false"
    )
    self_color_override_default = (
        "2" if bag_team == '0' else "0" if bag_team == '1' else "-1"
    )
    serial_port_default = str(pre_match_config.get('serial_port', _default_serial_port()))
    try:
        serial_baud_default = str(int(pre_match_config.get('serial_baud', 115200)))
    except (TypeError, ValueError):
        serial_baud_default = "115200"

    # 回放和实时共用同一套相机内参；bag.camera_params 已废弃。
    camera_params_path = _select_calibration_path(
        calibration_config,
        runtime_for_calibration,
        'camera_params',
        'src/tdt_vision/camera/config/hik.yaml',
    )
    # bag 优先用 bag.out_matrix_{red,blue} (回放录制时的相机姿态),
    # 否则回退到全局 calibration.out_matrix_*. 与 camera_params 一致地隔离 bag/比赛.
    bag_self_color = _effective_self_color(calibration_config, runtime_for_calibration)
    bag_out_key = 'out_matrix_red' if bag_self_color == 2 else \
                  'out_matrix_blue' if bag_self_color == 0 else None
    bag_out_override = bag_config.get(bag_out_key) if bag_out_key else None
    if not bag_out_override:
        bag_out_override = bag_config.get('out_matrix')
    if bag_out_override:
        out_matrix_path = str(bag_out_override)
        print(f"[bag.launch] Using bag-specific out_matrix: {out_matrix_path}")
    else:
        out_matrix_path = _select_calibration_path(
            calibration_config,
            runtime_for_calibration,
            'out_matrix',
            'config/out_matrix.yaml',
        )

    map_height = float(map_config.get('height', 15.0))
    icp_init = _camera_center_from_extrinsics(out_matrix_path, map_height)
    print(f"[bag.launch] ICP initial guess from {out_matrix_path}: "
          f"({icp_init[0]:.2f}, {icp_init[1]:.2f}, {icp_init[2]:.2f})")

    selected_color = _effective_self_color(calibration_config, runtime_for_calibration)
    show_image_default = "true" if bool(bag_config.get('show_image', False)) else "false"
    enable_lidar_pipeline_default = (
        "true" if bool(bag_config.get('enable_lidar_pipeline', True)) else "false"
    )
    enable_serial_default = "true" if bool(bag_config.get('enable_serial', False)) else "false"
    serial_dry_run_default = "true" if bool(bag_config.get('serial_dry_run', True)) else "false"
    serial_use_resolve_fallback_default = (
        "true"
        if bool(
            bag_config.get(
                'serial_use_resolve_fallback',
                not bool(bag_config.get('enable_lidar_pipeline', True)),
            )
        )
        else "false"
    )
    publish_match_info_default = (
        "true" if bool(bag_config.get('publish_match_info', False)) else "false"
    )
    enable_map_server_default = (
        "true" if bool(bag_config.get('enable_map_server', False)) else "false"
    )
    use_bag_timing_default = (
        "true" if bool(bag_config.get('use_bag_timing', True)) else "false"
    )
    wait_on_lidar_timing_default = (
        "true" if bool(bag_config.get('wait_on_lidar_timing', False)) else "false"
    )
    replay_rate_default = str(float(bag_config.get('replay_rate', 1.0)))
    max_sleep_ms_default = str(int(bag_config.get('max_sleep_ms', 300)))
    legacy_mode_default = (
        "true" if bool(bag_config.get('legacy_mode', True)) else "false"
    )
    legacy_cycle_ms_default = str(int(bag_config.get('legacy_cycle_ms', 100)))
    bag_team_source = "bag.team" if bag_config.get('team') is not None else "pre_match.team"
    print(
        f"[bag.launch] team={bag_team} (from {bag_team_source}), "
        f"effective_self_color={selected_color}, "
        f"camera_params={camera_params_path}, out_matrix={out_matrix_path}"
    )
    print(
        f"[bag.launch] defaults show_image={show_image_default}, "
        f"enable_lidar_pipeline={enable_lidar_pipeline_default}, enable_serial={enable_serial_default}, "
        f"publish_match_info={publish_match_info_default}, "
        f"serial_use_resolve_fallback={serial_use_resolve_fallback_default}, "
        f"input_is_self_frame={input_is_self_frame_default}, self_color_override={self_color_override_default}"
    )

    show_image_arg = DeclareLaunchArgument(
        "show_image",
        default_value=show_image_default,
        description="Whether detect node opens the cv window",
    )
    rosbag_file_arg = DeclareLaunchArgument(
        "rosbag_file",
        default_value=str(bag_config.get('rosbag_file', 'bags/latest')),
        description="Path to rosbag db3 file",
    )
    # Backward compatibility: keep old args as no-op so legacy commands do not fail.
    enable_foxglove_arg = DeclareLaunchArgument(
        "enable_foxglove",
        default_value="false",
        description="Deprecated and ignored. Foxglove has been removed from this launch.",
    )
    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="Deprecated and ignored.",
    )
    enable_lidar_pipeline_arg = DeclareLaunchArgument(
        "enable_lidar_pipeline",
        default_value=enable_lidar_pipeline_default,
        description="Enable lidar->cluster->kalman full replay chain",
    )
    enable_serial_arg = DeclareLaunchArgument(
        "enable_serial",
        default_value=enable_serial_default,
        description="Enable radar serial node in replay",
    )
    serial_dry_run_arg = DeclareLaunchArgument(
        "serial_dry_run",
        default_value=serial_dry_run_default,
        description="When replay serial enabled, dry_run=true publishes tx_raw only",
    )
    serial_use_resolve_fallback_arg = DeclareLaunchArgument(
        "serial_use_resolve_fallback",
        default_value=serial_use_resolve_fallback_default,
        description="Use /resolve_result to feed serial when lidar/kalman radar topic is unavailable",
    )
    input_is_self_frame_arg = DeclareLaunchArgument(
        "input_is_self_frame",
        default_value=input_is_self_frame_default,
        description="Whether replay input points are in self-view frame",
    )
    self_color_override_arg = DeclareLaunchArgument(
        "self_color_override",
        default_value=self_color_override_default,
        description="Force self color for normalization: -1 auto, 0 blue, 2 red",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value=serial_port_default,
        description="Serial port used when replay serial is enabled",
    )
    serial_baud_arg = DeclareLaunchArgument(
        "serial_baud",
        default_value=serial_baud_default,
        description="Serial baud rate used when replay serial is enabled",
    )
    publish_match_info_arg = DeclareLaunchArgument(
        "publish_match_info",
        default_value=publish_match_info_default,
        description="Replay /match_info from bag. Keep false for old bags with outdated MatchInfo layout.",
    )
    enable_map_server_arg = DeclareLaunchArgument(
        "enable_map_server",
        default_value=enable_map_server_default,
        description="Start nav2/map_server during bag replay",
    )
    use_bag_timing_arg = DeclareLaunchArgument(
        "use_bag_timing",
        default_value=use_bag_timing_default,
        description="Replay with recorded bag timing",
    )
    wait_on_lidar_timing_arg = DeclareLaunchArgument(
        "wait_on_lidar_timing",
        default_value=wait_on_lidar_timing_default,
        description="When using bag timing, also block on lidar timestamps",
    )
    replay_rate_arg = DeclareLaunchArgument(
        "replay_rate",
        default_value=replay_rate_default,
        description="Bag timing replay speed multiplier",
    )
    start_offset_sec_arg = DeclareLaunchArgument(
        "start_offset_sec",
        default_value=str(bag_config.get('start_offset_sec', 0.0)),
        description="Skip the first N seconds of the bag before publishing messages",
    )
    max_sleep_ms_arg = DeclareLaunchArgument(
        "max_sleep_ms",
        default_value=max_sleep_ms_default,
        description="Cap each bag-timing sleep; 0 means no cap",
    )
    legacy_mode_arg = DeclareLaunchArgument(
        "legacy_mode",
        default_value=legacy_mode_default,
        description="Use fixed-cycle replay pacing instead of recorded bag timing",
    )
    legacy_cycle_ms_arg = DeclareLaunchArgument(
        "legacy_cycle_ms",
        default_value=legacy_cycle_ms_default,
        description="Fixed replay delay applied after each camera frame in legacy mode",
    )


    def get_rosbag_player_node(package, plugin):
        player_params = {
            "rosbag_file": LaunchConfiguration("rosbag_file"),
            "loop_playback": bag_config.get('loop_playback', True),
            "use_bag_timing": ParameterValue(
                LaunchConfiguration("use_bag_timing"),
                value_type=bool,
            ),
            "replay_rate": ParameterValue(
                LaunchConfiguration("replay_rate"),
                value_type=float,
            ),
            "start_offset_sec": ParameterValue(
                LaunchConfiguration("start_offset_sec"),
                value_type=float,
            ),
            "max_sleep_ms": ParameterValue(
                LaunchConfiguration("max_sleep_ms"),
                value_type=int,
            ),
            "decode_compressed_image": bag_config.get('decode_compressed_image', True),
            "publish_compressed_image": bag_config.get('publish_compressed_image', False),
            "camera_max_fps": float(bag_config.get('camera_max_fps', 10.0)),
            "publish_lidar": ParameterValue(
                LaunchConfiguration("enable_lidar_pipeline"),
                value_type=bool,
            ),
            "publish_match_info": ParameterValue(
                LaunchConfiguration("publish_match_info"),
                value_type=bool,
            ),
            "wait_on_lidar_timing": ParameterValue(
                LaunchConfiguration("wait_on_lidar_timing"),
                value_type=bool,
            ),
            "legacy_mode": ParameterValue(
                LaunchConfiguration("legacy_mode"),
                value_type=bool,
            ),
            "legacy_cycle_ms": ParameterValue(
                LaunchConfiguration("legacy_cycle_ms"),
                value_type=int,
            ),
        }
        passthrough_topics = bag_config.get('passthrough_topics')
        if isinstance(passthrough_topics, list):
            safe_topics = [str(topic).strip() for topic in passthrough_topics if str(topic).strip()]
            if safe_topics:
                player_params["passthrough_topics"] = safe_topics

        return ComposableNode(
            package=package,
            plugin=plugin,
            name="rosbag_player_node",
            parameters=[player_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    def get_radar_detect_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name="radar_detect_node",
            parameters=[{
                "detect_view": ParameterValue(
                    LaunchConfiguration("show_image"),
                    value_type=bool,
                ),
                "force_draw_result": ParameterValue(
                    LaunchConfiguration("show_image"),
                    value_type=bool,
                ),
                "max_process_fps": float(bag_config.get('detect_max_process_fps', 0.0)),
                "detect_view_scale": 1.0,
                "drop_stale_ms": int(bag_config.get('detect_drop_stale_ms', 0)),
                # 3D cuboid overlay (axis-aligned in world frame; renders as
                # a parallelepiped in the perspective view).  Set draw_3d_box=false
                # in radar_runtime.yaml -> bag: to disable.
                "draw_3d_box": bag_config.get('draw_3d_box', True),
                "camera_params": camera_params_path,
                "out_matrix": out_matrix_path,
                "map_height": float(map_config.get('height', 15.0)),
                "calibration_points_in_referee_frame":
                    bool(calibration_config.get('points_in_referee_frame', True)),
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    def get_radar_resolve_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name="radar_resolve_node",
            parameters=[{
                'map_image': map_config.get('map_image', 'config/map/map.jpg'),
                'map_width': float(map_config.get('width', 28.0)),
                'map_height': float(map_config.get('height', 15.0)),
                'map_points': map_config.get('map_points', 'config/map/map_points.yaml'),
                'detect_use_legacy_plus_y': map_config.get('resolve_detect_use_legacy_plus_y', True),
                'camera_params': camera_params_path,
                'out_matrix': out_matrix_path,
                # resolve 调参写死为字面值 (radar_runtime.yaml 无 resolve: section)
                'pixel_scale_x':       1.0,
                'pixel_scale_y':       1.0,
                'frustum_pixel_gate':  150.0,
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    def get_camera_detector_container(
        radar_detect_node, radar_resolve_node, ros_bag_player_node
    ):
        return ComposableNodeContainer(
            name="camera_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                radar_detect_node,
                radar_resolve_node,
                ros_bag_player_node,
            ],
            output="both",
            emulate_tty=True,
        )

    def get_detect_debug_view_container():
        return ComposableNodeContainer(
            name="detect_debug_view_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="tdt_vision",
                    plugin="tdt_radar::DetectDebugView",
                    name="detect_debug_view",
                    parameters=[{
                        "image_topic": "detect_debug_image",
                        "window_name": "detect",
                    }],
                    extra_arguments=[{"use_intra_process_comms": False}],
                )
            ],
            output="both",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("show_image")),
        )

    radar_detect_node = get_radar_detect_node("tdt_vision", "tdt_radar::Detect")
    radar_resolve_node = get_radar_resolve_node("tdt_vision", "tdt_radar::Resolve")

    ros_bag_player_node = get_rosbag_player_node('rosbag_player', 'RosbagPlayer')

    cam_detector = get_camera_detector_container(
        radar_detect_node, radar_resolve_node, ros_bag_player_node
    )
    detect_debug_view_container = get_detect_debug_view_container()

    plugin_map_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_share_path,
                    "launch",
                    "map.launch.py",
                )
            ]
        ),
        condition=IfCondition(LaunchConfiguration("enable_map_server")),
    )
    lidar_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    lidar_pkg_share_path,
                    "launch",
                    "lidar.launch.py",
                )
            ]
        ),
        launch_arguments={
            "input_is_self_frame": LaunchConfiguration("input_is_self_frame"),
            "self_color_override": LaunchConfiguration("self_color_override"),
            "map_width": str(float(map_config.get('width', 28.0))),
            "map_height": str(float(map_config.get('height', 15.0))),
            "map_pcd": map_config.get('map_pcd', 'config/map/map.pcd'),
            "enable_kalman_aux_input": "false",
            "extrinsics_yaml": out_matrix_path,
            "map_height": str(map_height),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_lidar_pipeline")),
    )

    serial_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    serial_pkg_share_path,
                    "launch",
                    "radar_serial.launch.py",
                )
            ]
        ),
        launch_arguments={
            "port": LaunchConfiguration("serial_port"),
            "baud_rate": LaunchConfiguration("serial_baud"),
            "self_color_override": LaunchConfiguration("self_color_override"),
            "dry_run": LaunchConfiguration("serial_dry_run"),
            "enable_resolve_fallback": LaunchConfiguration("serial_use_resolve_fallback"),
            "map_width": str(float(map_config.get('width', 28.0))),
            "map_height": str(float(map_config.get('height', 15.0))),
            "enable_receive": "false",
            "publish_match_info": "false",
            "enable_decision": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_serial")),
    )

    return LaunchDescription(
        [
            show_image_arg,
            rosbag_file_arg,
            enable_foxglove_arg,
            foxglove_port_arg,
            enable_lidar_pipeline_arg,
            enable_serial_arg,
            serial_dry_run_arg,
            serial_use_resolve_fallback_arg,
            input_is_self_frame_arg,
            self_color_override_arg,
            serial_port_arg,
            serial_baud_arg,
            publish_match_info_arg,
            enable_map_server_arg,
            use_bag_timing_arg,
            wait_on_lidar_timing_arg,
            replay_rate_arg,
            start_offset_sec_arg,
            max_sleep_ms_arg,
            legacy_mode_arg,
            legacy_cycle_ms_arg,
            _system_libusb_preload_action(),
            cam_detector,
            detect_debug_view_container,
            plugin_map_launch_cmd,
            lidar_launch_cmd,
            serial_launch_cmd,
        ]
    )
