import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_serial_port():
    preferred = ['/dev/gimbal']
    for path in preferred:
        if os.path.exists(path):
            return path

    for base_dir in ('/dev/serial/by-id', '/dev/serial/by-path'):
        if os.path.isdir(base_dir):
            entries = sorted(os.listdir(base_dir))
            if entries:
                return os.path.join(base_dir, entries[0])

    for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*'):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]

    return '/dev/ttyUSB0'


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value=_default_serial_port(),
        description='Serial port path'
    )

    baud_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    send_hz_arg = DeclareLaunchArgument(
        'send_hz',
        default_value='5.0',
        description='Radar packet send rate (Hz)'
    )

    recv_hz_arg = DeclareLaunchArgument(
        'receive_hz',
        default_value='50.0',
        description='Receive poll rate (Hz)'
    )

    scale_arg = DeclareLaunchArgument(
        'coord_scale',
        default_value='100.0',
        description='Coordinate scale before uint16 encoding (meter->centimeter uses 100)'
    )

    map_width_arg = DeclareLaunchArgument(
        'map_width',
        default_value='28.0',
        description='Competition map width in meters; TX coordinates are clamped into [0, width]'
    )

    map_height_arg = DeclareLaunchArgument(
        'map_height',
        default_value='15.0',
        description='Competition map height in meters; TX coordinates are clamped into [0, height]'
    )

    match_info_topic_arg = DeclareLaunchArgument(
        'match_info_topic',
        default_value='/match_info',
        description='MatchInfo topic used to resolve self color'
    )

    self_color_override_arg = DeclareLaunchArgument(
        'self_color_override',
        default_value='-1',
        description='Force self color: -1 auto, 0 blue, 2 red'
    )

    tx_cmd_id_arg = DeclareLaunchArgument(
        'tx_cmd_id',
        default_value='773',
        description='TX command id in decimal (0x0305 = 773)'
    )

    radar_topic_arg = DeclareLaunchArgument(
        'radar_topic',
        default_value='/radar2sentry',
        description='Radar2Sentry topic to subscribe'
    )

    radar_topic_compat_arg = DeclareLaunchArgument(
        'radar_topic_compat',
        default_value='/Radar2Sentry',
        description='Optional compatibility topic, set equal to radar_topic to disable'
    )

    enable_resolve_fallback_arg = DeclareLaunchArgument(
        'enable_resolve_fallback',
        default_value='false',
        description='Use /resolve_result as fallback input when /radar2sentry is missing/stale'
    )

    resolve_topic_arg = DeclareLaunchArgument(
        'resolve_topic',
        default_value='/resolve_result',
        description='DetectResult topic used by resolve fallback'
    )

    kalman_topic_arg = DeclareLaunchArgument(
        'kalman_topic',
        default_value='/kalman_detect',
        description='DetectResult topic used by fused fallback'
    )

    map_points_topic_arg = DeclareLaunchArgument(
        'map_points_topic',
        default_value='/debug_map_points',
        description='DetectResult topic published by debug_map final display cache'
    )

    radar_topic_timeout_arg = DeclareLaunchArgument(
        'radar_topic_timeout_s',
        default_value='0.5',
        description='When radar_topic data age exceeds this threshold, fallback can be used'
    )

    fallback_slot_hold_arg = DeclareLaunchArgument(
        'fallback_slot_hold_s',
        default_value='30.0',
        description='Seconds to keep last valid per-slot fallback coordinate'
    )

    rx_cmd_id_arg = DeclareLaunchArgument(
        'rx_expect_cmd_id',
        default_value='771',
        description='Expected RX command id in decimal (0x0303 = 771)'
    )

    enable_receive_arg = DeclareLaunchArgument(
        'enable_receive',
        default_value='true',
        description='Enable serial receive loop'
    )

    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='Do not use serial device, publish TX packets to /radar/tx_raw for validation'
    )

    log_rx_hex_arg = DeclareLaunchArgument(
        'log_rx_payload_hex',
        default_value='false',
        description='Log RX payload hex when expected cmd id is received'
    )

    enable_decision_arg = DeclareLaunchArgument(
        'enable_decision',
        default_value='true',
        description='Enable decision packet send (0x0301 with content 0x0121)'
    )

    decision_topic_arg = DeclareLaunchArgument(
        'decision_topic',
        default_value='/radar/decision_request',
        description='UInt8 topic; each message triggers one decision packet send'
    )

    decision_receiver_arg = DeclareLaunchArgument(
        'decision_receiver_id',
        default_value='32896',
        description='Decision packet receiver id in decimal (0x8080 = 32896)'
    )

    decision_content_arg = DeclareLaunchArgument(
        'decision_content_id',
        default_value='289',
        description='Decision content id in decimal (0x0121 = 289)'
    )

    decision_interval_arg = DeclareLaunchArgument(
        'decision_min_interval_s',
        default_value='0.10',
        description='Minimum interval between two decision packet sends'
    )

    publish_match_info_arg = DeclareLaunchArgument(
        'publish_match_info',
        default_value='true',
        description='Publish parsed referee downlink to /match_info'
    )

    match_info_timeout_arg = DeclareLaunchArgument(
        'match_info_timeout_s',
        default_value='2.0',
        description='Set match_time=-200 when referee data timeout exceeds this value'
    )

    node = Node(
        package='kalman_filter',
        executable='radar_serial_node',
        name='radar_serial_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
            'send_hz': ParameterValue(LaunchConfiguration('send_hz'), value_type=float),
            'receive_hz': ParameterValue(LaunchConfiguration('receive_hz'), value_type=float),
            'coord_scale': ParameterValue(LaunchConfiguration('coord_scale'), value_type=float),
            'map_width': ParameterValue(LaunchConfiguration('map_width'), value_type=float),
            'map_height': ParameterValue(LaunchConfiguration('map_height'), value_type=float),
            'match_info_topic': LaunchConfiguration('match_info_topic'),
            'self_color_override': ParameterValue(LaunchConfiguration('self_color_override'), value_type=int),
            'tx_cmd_id': ParameterValue(LaunchConfiguration('tx_cmd_id'), value_type=int),
            'radar_topic': LaunchConfiguration('radar_topic'),
            'radar_topic_compat': LaunchConfiguration('radar_topic_compat'),
            'enable_resolve_fallback': ParameterValue(
                LaunchConfiguration('enable_resolve_fallback'), value_type=bool
            ),
            'resolve_topic': LaunchConfiguration('resolve_topic'),
            'kalman_topic': LaunchConfiguration('kalman_topic'),
            'map_points_topic': LaunchConfiguration('map_points_topic'),
            'radar_topic_timeout_s': ParameterValue(
                LaunchConfiguration('radar_topic_timeout_s'), value_type=float
            ),
            'fallback_slot_hold_s': ParameterValue(
                LaunchConfiguration('fallback_slot_hold_s'), value_type=float
            ),
            'rx_expect_cmd_id': ParameterValue(LaunchConfiguration('rx_expect_cmd_id'), value_type=int),
            'enable_receive': ParameterValue(LaunchConfiguration('enable_receive'), value_type=bool),
            'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
            'log_rx_payload_hex': ParameterValue(LaunchConfiguration('log_rx_payload_hex'), value_type=bool),
            'enable_decision': ParameterValue(LaunchConfiguration('enable_decision'), value_type=bool),
            'decision_topic': LaunchConfiguration('decision_topic'),
            'decision_receiver_id': ParameterValue(LaunchConfiguration('decision_receiver_id'), value_type=int),
            'decision_content_id': ParameterValue(LaunchConfiguration('decision_content_id'), value_type=int),
            'decision_min_interval_s': ParameterValue(LaunchConfiguration('decision_min_interval_s'), value_type=float),
            'publish_match_info': ParameterValue(LaunchConfiguration('publish_match_info'), value_type=bool),
            'match_info_timeout_s': ParameterValue(LaunchConfiguration('match_info_timeout_s'), value_type=float),
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        send_hz_arg,
        recv_hz_arg,
        scale_arg,
        map_width_arg,
        map_height_arg,
        match_info_topic_arg,
        self_color_override_arg,
        tx_cmd_id_arg,
        radar_topic_arg,
        radar_topic_compat_arg,
        enable_resolve_fallback_arg,
        resolve_topic_arg,
        kalman_topic_arg,
        map_points_topic_arg,
        radar_topic_timeout_arg,
        fallback_slot_hold_arg,
        rx_cmd_id_arg,
        enable_receive_arg,
        dry_run_arg,
        log_rx_hex_arg,
        enable_decision_arg,
        decision_topic_arg,
        decision_receiver_arg,
        decision_content_arg,
        decision_interval_arg,
        publish_match_info_arg,
        match_info_timeout_arg,
        node,
    ])
