"""
轻量视觉测试 launch：用本地 MP4 视频驱动 YOLO 检测，不依赖相机/激光/bag。

用法：
  ros2 launch tdt_vision video.launch.py video:=videos/7.26_3072px.mp4
  ros2 launch tdt_vision video.launch.py video:=videos/7.26_3072px.mp4 rate:=1.0 start:=30.0
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    video_arg = DeclareLaunchArgument(
        'video',
        default_value='videos/7.26_3072px.mp4',
        description='Path to MP4 video file'
    )
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback speed factor (1.0 = real-time)'
    )
    start_arg = DeclareLaunchArgument(
        'start',
        default_value='0.0',
        description='Start offset in seconds'
    )
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='3072',
        description='Publish width (0 = original). 3072 matches hik.yaml intrinsics'
    )

    detect_node = ComposableNode(
        package='tdt_vision',
        plugin='tdt_radar::Detect',
        name='radar_detect_node',
        parameters=[{
            'detect_view': True,
            'force_draw_result': True,
            'detect_view_scale': 1.0,
            'sahi_enable': True,
            'sahi_scale': 1.0,
            'sahi_overlap': 0.2,
            'max_process_fps': 0.0,
            'drop_stale_ms': 0,
        }],
        extra_arguments=[{'use_intra_process_comms': False}]
    )

    container = ComposableNodeContainer(
        name='video_detect_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[detect_node],
        output='screen',
    )

    video_publisher = ExecuteProcess(
        cmd=[
            'python3', 'tools/video_publisher.py',
            LaunchConfiguration('video'),
            '--rate-factor', LaunchConfiguration('rate'),
            '--width',       LaunchConfiguration('width'),
            '--start',       LaunchConfiguration('start'),
        ],
        cwd=os.environ.get('ROS_WORKSPACE', os.path.expanduser('~/T')),
        output='screen',
    )

    return LaunchDescription([
        video_arg, rate_arg, start_arg, width_arg,
        container,
        video_publisher,
    ])
