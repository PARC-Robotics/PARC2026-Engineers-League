"""Start the referee and dashboard; optionally record the /parc topics to a rosbag.

ros2 launch parc_referee referee.launch.py judge_token:=<secret> odom_topic:=/odom record:=true
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_dir = os.path.expanduser(
        f'~/parc_runs/bags/{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    args = {
        'judge_token': 'parc2026',
        'odom_topic': '/odom',
        'web_port': '8080',
        'time_limit_s': '600.0',
        'hold_time_s': '3.0',
        'record': 'true',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] + [
            Node(
                package='parc_referee', executable='referee', name='parc_referee', output='screen',
                parameters=[{
                    'judge_token': LaunchConfiguration('judge_token'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'web_port': LaunchConfiguration('web_port'),
                    'time_limit_s': LaunchConfiguration('time_limit_s'),
                    'hold_time_s': LaunchConfiguration('hold_time_s'),
                }],
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('record')),
                cmd=['ros2', 'bag', 'record', '-o', bag_dir,
                     '/parc/tag_order', '/parc/start', '/parc/tag_visit',
                     '/parc/finish', '/parc/run_status', LaunchConfiguration('odom_topic')],
                output='screen',
            ),
        ])
