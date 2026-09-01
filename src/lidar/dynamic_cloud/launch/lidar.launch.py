import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
    # 所有 cluster / kalman 算法参数都已写死为字面值, 不再从 YAML 读取;
    # 调参直接编辑本文件, 或运行时 `ros2 param set /kalman_filter_node <name> <v>`。
    # radar_runtime.yaml 只承载赛前一次性配置 (pre_match / map / bag 等)。
    pre_match_config = runtime_config.get('pre_match', {})
    pre_match_team = str(pre_match_config.get('team', '')).strip()
    _default_self_color = '2' if pre_match_team == '0' else '0' if pre_match_team == '1' else '-1'
    _default_self_frame = str(pre_match_config.get('self_frame', False)).lower()

    input_is_self_frame_arg = DeclareLaunchArgument(
        'input_is_self_frame',
        default_value=_default_self_frame,
        description='Whether the input is in self-view frame'
    )

    self_color_override_arg = DeclareLaunchArgument(
        'self_color_override',
        default_value=_default_self_color,
        description='Force self color: -1 auto, 0 blue, 2 red'
    )

    # map_width / map_height / map_pcd 均为 RM2026 场地常量, 已在各节点代码内写死。
    # kalman_filter 仍需 map_width/height 作为参数 (赋值为常量、依然走 declare_parameter)。

    enable_kalman_aux_input_arg = DeclareLaunchArgument(
        'enable_kalman_aux_input',
        default_value='true',
        description='Enable kalman camera/match auxiliary inputs (/resolve_result, /match_info)'
    )

    extrinsics_yaml_arg = DeclareLaunchArgument(
        'extrinsics_yaml', default_value='',
        description='Path to solvePnP extrinsics YAML for ICP initial pose')
    map_height_arg = DeclareLaunchArgument(
        'map_height', default_value='15.0',
        description='Field height in metres (for legacy→referee Y conversion)')

    localization_node = ComposableNode(
        package='localization',
        plugin='tdt_radar::Localization',
        name='localization_node',
        parameters=[{
            'extrinsics_yaml': LaunchConfiguration('extrinsics_yaml'),
            'map_height': ParameterValue(LaunchConfiguration('map_height'), value_type=float),
        }],
        extra_arguments=[{'use_multi_threaded_executor': True}],
    )

    # dynamic_cloud 仅 self_color 走参数 (由 self_color_override 驱动); ROI / dart / fly
    # 几何 / arena 尺寸 / map_pcd 全部写死在 .cpp 内。
    dynamic_cloud_node = ComposableNode(
        package='dynamic_cloud',
        plugin='tdt_radar::DynamicCloud',
        name='dynamic_cloud_node',
        parameters=[{
            'self_color': ParameterValue(LaunchConfiguration('self_color_override'), value_type=int),
        }],
    )

    # cluster_node 几何常量已全部写死 (RM2026 规则手册), 仅保留 timing log 开关。
    cluster_node = ComposableNode(
        package='cluster',
        plugin='tdt_radar::Cluster',
        name='cluster_node',
        parameters=[{
            'log_cluster_timing': False,
        }],
    )

    # kalman_filter 调参全部写死为字面值; 比赛前微调请直接编辑本块。
    # 运行时可用 `ros2 param set /kalman_filter_node <name> <v>` 临时改动 (重启失效)。
    kalman_filter_node = ComposableNode(
        package='kalman_filter',
        plugin='tdt_radar::KalmanFilter',
        name='kalman_filter_node',
        parameters=[{
            # arena 尺寸 = RM2026 场地常量
            'map_width':  28.0,
            'map_height': 15.0,
            'input_is_self_frame': ParameterValue(
                LaunchConfiguration('input_is_self_frame'),
                value_type=bool,
            ),
            'self_color_override': ParameterValue(
                LaunchConfiguration('self_color_override'),
                value_type=int,
            ),
            'enable_aux_input': ParameterValue(
                LaunchConfiguration('enable_kalman_aux_input'),
                value_type=bool,
            ),
            # ── 匹配 / 跟踪 ─────────────────────────────────────────────
            # pos_reinforce: 基于规则手册的位置强化 (引入预判偏置), 关掉.
            'pos_reinforce_enable':    False,
            'lidar_match_radius':      0.8,
            'lidar_match_speed_gain':  4.0,
            # camera_time_threshold_s: 多久无观测才允许 KF 接管.
            # 调长到 3s, 避免短暂遮挡时 KF 启动 goal-attraction 拉到假位置.
            'camera_time_threshold_s': 3.0,
            'track_timeout_s':         2.0,
            'track_history_size':      20,
            'blind_timeout_s':         10.0,
            'nms_merge_r':             0.8,
            # ── 关闭预判机制 ─────────────────────────────────────────────
            # guess_pts: 盲区时 snap 到 config/guess_pts.yaml 预设位置.
            # shadow:    阴影图触发即时猜点.
            # 两者会让 minimap 显示与相机实际位置不一致的 "预判位置". 关掉.
            'guess_pts_enable':        False,
            'shadow_enable':           False,
            # ── 静止过滤 ────────────────────────────────────────────────
            'static_filter_dist':  0.3,
            'static_filter_age_s': 3.0,
            # ── 颜色推断 ────────────────────────────────────────────────
            'spatial_color_mode':  False,
            # ── ID 评分 (HKUST 风格历史投票) ────────────────────────────
            'id_score_inc':     40.0,
            'id_score_dec':      2.0,
            'id_score_decay':    5.0,
            'id_score_thresh':  60.0,
            'id_score_hi_mult':  0.1,
            # ── KF 协方差冻结 (短时遮挡稳定) ────────────────────────────
            'kf_cov_factor':   0.002,
            'kf_cov_freeze_s': 3.0,
            # ── Ghost 继承 ──────────────────────────────────────────────
            'ghost_inherit_r': 1.0,
            'ghost_ttl_s':     8.0,
            # 注: cost_w_hist/botid/pos 与 guess_*  在 KF 内部 declare_parameter,
            #     运行时可调 (默认: hist=0.45 / botid=0.15 / pos=1.0,
            #     guess: d=0.08 cos=0.20 snap_max=0.60 snap_ramp=0.15 snap_start_s=2.0)。
        }],
    )

    localization_container = ComposableNodeContainer(
        name='localization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            localization_node,
        ],
        output='both',
        emulate_tty=True,
    )

    dynamic_cloud_container = ComposableNodeContainer(
        name='dynamic_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[dynamic_cloud_node],
        output='both',
        emulate_tty=True,
    )

    cluster_container = ComposableNodeContainer(
        name='cluster_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[cluster_node],
        output='both',
        emulate_tty=True,
    )

    kalman_container = ComposableNodeContainer(
        name='kalman_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[kalman_filter_node],
        output='both',
        emulate_tty=True,
    )

    return LaunchDescription([
        input_is_self_frame_arg,
        self_color_override_arg,
        enable_kalman_aux_input_arg,
        extrinsics_yaml_arg,
        map_height_arg,
        localization_container,
        dynamic_cloud_container,
        cluster_container,
        kalman_container,
    ])
