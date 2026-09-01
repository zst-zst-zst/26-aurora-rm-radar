import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def _load_runtime_config():
    candidates = [
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), '../../../../config/radar_runtime.yaml')),
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), '../../../../../config/radar_runtime.yaml')),
    ]
    for config_path in candidates:
        if os.path.exists(config_path):
            with open(config_path, 'r', encoding='utf-8') as file:
                return yaml.safe_load(file) or {}
    raise FileNotFoundError('Cannot find config/radar_runtime.yaml from launch path')


def generate_launch_description():
    ld = LaunchDescription()
    runtime_config = _load_runtime_config()
    map_path = runtime_config.get('map', {}).get('map_yaml')
    if not map_path:
        map_path = 'config/map/map.yaml'

    map_server_node = Node(
        package="nav2_map_server",
        executable='map_server',
        output='screen', emulate_tty=True,
        parameters=[{'yaml_filename': map_path,'frame_id':'rm_frame'}]
    )
    activate_map_server = ExecuteProcess(
        cmd=['bash', '-lc',
             'source /opt/ros/jazzy/setup.bash\n'
             'echo "等待 map_server 生命周期服务"\n'
             'for i in $(seq 1 30); do\n'
             '  if ros2 lifecycle get /map_server >/dev/null 2>&1; then\n'
             '    break\n'
             '  fi\n'
             '  sleep 1\n'
             'done\n'
             'echo "指令激活 map_server"\n'
             'ros2 lifecycle set /map_server configure >/dev/null 2>&1 || true\n'
             'ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true\n'
             'ros2 lifecycle get /map_server || true\n'
             'echo "激活 map_server 完成"\n'
             ],
        name='configure_map_server',
        output='screen'
    )
    ld.add_action(map_server_node)
    ld.add_action(activate_map_server)

    return ld
