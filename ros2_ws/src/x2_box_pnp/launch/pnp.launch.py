import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('x2_box_pnp')
    return LaunchDescription([
        DeclareLaunchArgument('pnp_params', default_value=os.path.join(share, 'config', 'pnp_params.yaml')),
        DeclareLaunchArgument('keyframes_file', default_value=os.path.join(share, 'config', 'arm_keyframes.yaml')),
        DeclareLaunchArgument('auto_start', default_value='false'),
        Node(package='x2_box_pnp', executable='pnp_fsm_node', name='pnp_fsm', output='screen',
             parameters=[LaunchConfiguration('pnp_params'),
                         {'keyframes_file': LaunchConfiguration('keyframes_file'),
                          'auto_start': LaunchConfiguration('auto_start')}]),
    ])
