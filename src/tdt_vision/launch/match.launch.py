import os
import sys

os.environ.setdefault('ALLUSERSPROFILE', os.path.abspath(os.path.join(os.getcwd(), 'GenICamCache')))
from datetime import datetime

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, os.path.dirname(__file__))
from _common import (
    BRAND_CAMERA_PARAMS as _BRAND_CAMERA_PARAMS,
    apply_team_override,
    default_serial_port as _default_serial_port,
    effective_brand as _effective_brand,
    effective_self_color as _effective_self_color,
    load_runtime_config,
    resolve_workspace_config_dir,
    select_calibration_path as _select_calibration_path,
    system_libusb_preload_action as _system_libusb_preload_action,
    warn_if_same_side_calibration,
)


def _include(pkg_name: str, launch_file: str, launch_arguments=None, condition=None):
    pkg_share = get_package_share_directory(pkg_name)
    launch_path = os.path.join(pkg_share, 'launch', launch_file)
    kwargs = {
        'launch_description_source': PythonLaunchDescriptionSource(launch_path),
    }
    if launch_arguments:
        kwargs['launch_arguments'] = launch_arguments.items()
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(**kwargs)


def _load_runtime_config():
    return load_runtime_config(__file__)


def _resolve_workspace_config_dir():
    return resolve_workspace_config_dir(__file__)


def _launch_arg_value(name, default):
    prefix = f'{name}:='
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def _warn_if_same_side_calibration(calibration_config):
    warn_if_same_side_calibration(
        calibration_config,
        '2.launch',
        keys=('calibrate_points', 'camera_params', 'out_matrix'),
    )


def _require_serial_device(context):
    enabled = str(LaunchConfiguration('serial_enable').perform(context)).strip().lower()
    if enabled not in ('1', 'true', 'yes', 'on'):
        return []
    port = str(LaunchConfiguration('port').perform(context)).strip()
    if not os.path.exists(port):
        raise RuntimeError(
            f"serial_enable=1 but serial port '{port}' does not exist. "
            "比赛必须有裁判系统串口: 先确认 USB-TTL 已插入并且 `ls /dev/gimbal` 存在；"
            "只做无串口调试时才使用 serial_enable:=0。"
        )
    return []


def _get_camera_node(camera_config_file, brand_override=None):
    params = [camera_config_file, {'view_local': False}]
    if brand_override:
        params.append({'brand': brand_override})
    return ComposableNode(
        package='tdt_vision',
        plugin='tdt_vision::NodeCamera',
        name='vision_camera_node',
        parameters=params,
        extra_arguments=[{'use_intra_process_comms': False}]
    )


def _get_camera_container(camera_node):
    """Camera runs in its own single-threaded container so images are
    delivered via DDS inter-process to the detector container.  This avoids
    intra-process threading issues with component_container_mt."""
    return ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[camera_node],
        output='both',
        emulate_tty=True,
    )


def _get_radar_detect_node(detect_view, max_process_fps,
                           camera_params_path, out_matrix_path,
                           map_height, calibration_referee_frame):
    return ComposableNode(
        package='tdt_vision',
        plugin='tdt_radar::Detect',
        name='radar_detect_node',
        parameters=[{
            'detect_view': detect_view,
            'force_draw_result': detect_view,
            'detect_view_scale': 1.0,
            'drop_stale_ms': 200,
            'max_process_fps': float(max_process_fps),
            # 3D cuboid overlay (与 bag 模式一致): 必须传 out_matrix 才会启用
            'draw_3d_box': True,
            'camera_params': camera_params_path,
            'out_matrix': out_matrix_path,
            'map_height': float(map_height),
            'calibration_points_in_referee_frame': bool(calibration_referee_frame),
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )


def _get_radar_resolve_node(map_config, camera_params_path, out_matrix_path):
    # resolve 调参全部写死: pixel_scale / frustum_pixel_gate / field_*
    # 赛前微调请直接改本块; map_* 仍从 YAML map: section 读。
    return ComposableNode(
        package='tdt_vision',
        plugin='tdt_radar::Resolve',
        name='radar_resolve_node',
        parameters=[{
            'map_image': map_config.get('map_image', 'config/map/map.jpg'),
            'map_width': float(map_config.get('width', 28.0)),
            'map_height': float(map_config.get('height', 15.0)),
            'map_points': map_config.get('map_points', 'config/map/map_points.yaml'),
            'detect_use_legacy_plus_y': True,
            'camera_params': camera_params_path,
            'out_matrix': out_matrix_path,
            'pixel_scale_x':       1.0,
            'pixel_scale_y':       1.0,
            'frustum_pixel_gate':  150.0,
            'field_grid_path':     'config/map/field_mesh.bin',
            'field_mesh_path':     '',
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )


def _get_record_node(image_record_hz):
    return ComposableNode(
        package='databag_tool',
        plugin='BagRecorderNode',
        name='record_node',
        parameters=[{
            'image_record_hz': float(image_record_hz),
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )


def _get_detector_container(radar_detect_node, radar_resolve_node,
                            record_node):
    nodes = [
        radar_detect_node,
        radar_resolve_node,
        record_node,
    ]
    nodes = [n for n in nodes if n is not None]
    return ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='both',
        emulate_tty=True,
    )


def _get_detect_debug_view_node():
    return ComposableNode(
        package='tdt_vision',
        plugin='tdt_radar::DetectDebugView',
        name='detect_debug_view',
        parameters=[{
            'image_topic': 'detect_debug_image',
            'window_name': 'detect',
        }],
        extra_arguments=[{'use_intra_process_comms': False}]
    )


def _get_detect_debug_view_container(debug_view_node):
    return ComposableNodeContainer(
        name='detect_debug_view_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[debug_view_node],
        output='both',
        emulate_tty=True,
    )


def generate_launch_description():
    pkg_share_path = get_package_share_directory('tdt_vision')
    workspace_config_dir = _resolve_workspace_config_dir()
    runtime_config = _load_runtime_config()

    # ── 每次启动强制重新 refine ─────────────────────────────────────────
    # 雷达整机每场都会换位置 (相机+LiDAR 相对位置不变), 旧的 map_refined.pcd
    # 是上次摆位下扫到的稀疏点云, 新位置下 ICP 会因 inlier 不足而拒匹配.
    # 启动前删掉, 让 localization 从 CAD map.pcd 重新冷启动 (~15s refine).
    refined_pcd = os.path.join(workspace_config_dir, 'map', 'map_refined.pcd')
    if os.path.exists(refined_pcd):
        try:
            os.remove(refined_pcd)
            print(f"[match.launch] Removed stale {refined_pcd} — will refine from CAD map.")
        except OSError as e:
            print(f"[match.launch][WARN] Failed to remove {refined_pcd}: {e}")
    pre_match_config = runtime_config.get('pre_match', {})
    map_config = runtime_config.get('map', {})
    calibration_config = runtime_config.get('calibration', {})
    runtime_overrides = runtime_config.get('runtime', {})
    _warn_if_same_side_calibration(calibration_config)

    effective_brand = _effective_brand(pre_match_config)

    runtime_for_calibration = dict(runtime_overrides)
    pre_match_team = str(_launch_arg_value(
        'team', str(pre_match_config.get('team', ''))
    )).strip()
    if pre_match_team == '0':
        runtime_for_calibration['self_color_override'] = 2
    elif pre_match_team == '1':
        runtime_for_calibration['self_color_override'] = 0

    # brand 自动推导 camera_params：始终以 pre_match.brand 为准覆盖内参路径
    brand_params_path = _BRAND_CAMERA_PARAMS.get(effective_brand)
    if brand_params_path:
        calibration_config = dict(calibration_config)
        calibration_config['camera_params_red'] = brand_params_path
        calibration_config['camera_params_blue'] = brand_params_path

    camera_params_path = _select_calibration_path(
        calibration_config,
        runtime_for_calibration,
        'camera_params',
        'config/camera_params.yaml',
    )
    out_matrix_path = _select_calibration_path(
        calibration_config,
        runtime_for_calibration,
        'out_matrix',
        'config/out_matrix.yaml',
    )
    selected_color = _effective_self_color(calibration_config, runtime_for_calibration)
    print(
        f"[match.launch] team={'red' if selected_color==2 else 'blue'}, "
        f"brand={effective_brand}, camera=night, "
        f"camera_params={camera_params_path}, out_matrix={out_matrix_path}"
    )

    team_arg = DeclareLaunchArgument(
        'team',
        default_value=str(pre_match_config.get('team', 1)),
        description='Team side: 0 red, 1 blue'
    )

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='night',
        description='Camera profile: fixed to night'
    )

    self_frame_arg = DeclareLaunchArgument(
        'self_frame',
        default_value=str(pre_match_config.get('self_frame', 1)),
        description='Input frame: 1 self-frame, 0 global-frame'
    )

    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value=str(pre_match_config.get('serial_port', _default_serial_port())),
        description='Serial port path'
    )

    serial_baud_arg = DeclareLaunchArgument(
        'baud',
        default_value=str(pre_match_config.get('serial_baud', 115200)),
        description='Serial baud rate'
    )

    serial_enable_arg = DeclareLaunchArgument(
        'serial_enable',
        default_value=str(pre_match_config.get('serial_enable', 1)),
        description='Enable serial node: 1 enable, 0 disable'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=str(pre_match_config.get('enable_map_debug', 0)),
        description='Start map debug: 1 enable, 0 disable'
    )

    lidar_enable_arg = DeclareLaunchArgument(
        'lidar_enable',
        default_value=str(pre_match_config.get('lidar_enable', 1)),
        description='Enable lidar stack (livox + lidar pipeline): 1 enable, 0 disable'
    )
    auto_record_arg = DeclareLaunchArgument(
        'auto_record',
        default_value=str(pre_match_config.get('enable_auto_record', 1)),
        description='Enable automatic bag recording: 1 enable, 0 disable'
    )

    # User simplified mapping:
    # team=0(red) -> self_color_override=2
    # team=1(blue) -> self_color_override=0
    self_color_override = PythonExpression([
        "'2' if '", LaunchConfiguration('team'), "' == '0' else '0'"
    ])

    input_is_self_frame = PythonExpression([
        "'true' if '", LaunchConfiguration('self_frame'), "' == '1' else 'false'"
    ])

    enable_debug_map = PythonExpression([
        "'true' if '", LaunchConfiguration('map'), "' == '1' else 'false'"
    ])

    enable_serial = PythonExpression([
        "'true' if '", LaunchConfiguration('serial_enable'), "' == '1' else 'false'"
    ])

    enable_lidar = PythonExpression([
        "'true' if '", LaunchConfiguration('lidar_enable'), "' == '1' else 'false'"
    ])

    serial_use_resolve_fallback = 'true'

    livox_launch = _include(
        'livox_ros2_driver',
        'livox_lidar_launch.py',
        condition=IfCondition(enable_lidar)
    )

    lidar_launch = _include(
        'dynamic_cloud',
        'lidar.launch.py',
        {
            'input_is_self_frame': input_is_self_frame,
            'self_color_override': self_color_override,
            # 用相机 solvePnP 外参做 ICP 初始位姿 (相机与 LiDAR 物理距离 <20cm,
            # 直接借用相机在 rm_frame 的位置作为 LiDAR 初始猜测, 冷启动只需 1~2 帧收敛)
            'extrinsics_yaml': out_matrix_path,
            'map_height': str(float(map_config.get('height', 15.0))),
        },
        condition=IfCondition(enable_lidar)
    )

    camera_config_file = os.path.join(workspace_config_dir, 'camera_driver_night.yaml')

    camera_node = _get_camera_node(camera_config_file, brand_override=effective_brand)
    radar_detect_node = _get_radar_detect_node(
        detect_view=True,
        max_process_fps=pre_match_config.get('detect_max_process_fps', 30.0),
        camera_params_path=camera_params_path,
        out_matrix_path=out_matrix_path,
        map_height=map_config.get('height', 15.0),
        calibration_referee_frame=calibration_config.get('points_in_referee_frame', True),
    )
    radar_resolve_node = _get_radar_resolve_node(
        map_config, camera_params_path, out_matrix_path
    )

    has_databag_tool = False
    auto_record_value = _launch_arg_value(
        'auto_record', str(pre_match_config.get('enable_auto_record', 1))
    )
    enable_auto_record = str(auto_record_value).strip().lower() in (
        '1', 'true', 'yes', 'on'
    )
    try:
        get_package_share_directory('databag_tool')
        if enable_auto_record:
            record_node = _get_record_node(pre_match_config.get('image_record_hz', 10.0))
            has_databag_tool = True
        else:
            record_node = None
    except PackageNotFoundError:
        record_node = None

    camera_container = _get_camera_container(camera_node)
    detector_container = _get_detector_container(
        radar_detect_node,
        radar_resolve_node,
        record_node,
    )
    detect_debug_view_node = _get_detect_debug_view_node()
    detect_debug_view_container = _get_detect_debug_view_container(detect_debug_view_node)

    serial_launch = _include(
        'kalman_filter',
        'radar_serial.launch.py',
        {
            'port': LaunchConfiguration('port'),
            'baud_rate': LaunchConfiguration('baud'),
            'self_color_override': self_color_override,
            'enable_resolve_fallback': serial_use_resolve_fallback,
            'map_width': str(float(map_config.get('width', 28.0))),
            'map_height': str(float(map_config.get('height', 15.0))),
        },
        condition=IfCondition(enable_serial)
    )

    map_launch = _include(
        'debug_map',
        'map.launch.py',
        {
            # debug_map inputs are already normalized to global in kalman/resolve path.
            # Passing self_frame here can introduce a second mirror transform.
            'input_is_self_frame': 'false',
            'self_color_override': self_color_override,
        },
        condition=IfCondition(enable_debug_map)
    )

    auto_record_enabled = enable_auto_record
    record_dir = str(pre_match_config.get('record_dir', '/home/zst/T/bags')).strip()
    record_prefix = str(pre_match_config.get('record_prefix', 'match')).strip()
    record_topics = pre_match_config.get(
        'record_topics',
        ['/livox/lidar', '/compressed_image', '/match_info']
    )
    if not isinstance(record_topics, list) or not record_topics:
        record_topics = ['/livox/lidar', '/compressed_image', '/match_info']

    safe_topics = [str(topic).strip() for topic in record_topics if str(topic).strip()]
    if not safe_topics:
        safe_topics = ['/livox/lidar', '/compressed_image', '/match_info']

    # Guard rail: image + lidar + match_info are mandatory for replay judgement.
    mandatory_topics = ['/livox/lidar', '/compressed_image', '/match_info']
    for topic in mandatory_topics:
        if topic not in safe_topics:
            safe_topics.append(topic)

    bag_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_output = os.path.join(record_dir, f'{record_prefix}_{bag_time}')
    latest_link = os.path.join(record_dir, 'latest')
    record_cmd = (
        f'mkdir -p "{record_dir}" && '
        f'if [ -L "{latest_link}" ]; then rm "{latest_link}"; '
        f'elif [ -e "{latest_link}" ]; then mv "{latest_link}" "{latest_link}.preserved"; fi && '
        f'ln -s "{bag_output}" "{latest_link}" && '
        f'ros2 bag record {" ".join(safe_topics)} '
        f'-o "{bag_output}"'
    )
    use_fallback_record = auto_record_enabled and (not has_databag_tool)

    auto_record_process = ExecuteProcess(
        cmd=['bash', '-lc', record_cmd],
        output='screen',
        condition=IfCondition('true' if use_fallback_record else 'false')
    )
    print(
        f"[match.launch] recorder={'databag_tool' if has_databag_tool else 'ros2_bag_fallback'}, "
        f"auto_record={'on' if auto_record_enabled else 'off'}, "
        f"output={bag_output}, latest={latest_link}, topics={safe_topics}"
    )

    return LaunchDescription([
        team_arg,
        camera_arg,
        self_frame_arg,
        serial_port_arg,
        serial_baud_arg,
        serial_enable_arg,
        map_arg,
        lidar_enable_arg,
        auto_record_arg,
        OpaqueFunction(function=_require_serial_device),
        _system_libusb_preload_action(),
        livox_launch,
        lidar_launch,
        camera_container,
        detector_container,
        detect_debug_view_container,
        serial_launch,
        map_launch,
        auto_record_process,
    ])
