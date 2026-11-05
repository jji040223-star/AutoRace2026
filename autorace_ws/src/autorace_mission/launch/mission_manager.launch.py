"""미션 매니저 (FSM).

  ros2 launch autorace_mission mission_manager.launch.py                      # 시뮬
  ros2 launch autorace_mission mission_manager.launch.py use_sim_time:=false  # 실차
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('autorace_mission'), 'config', 'mission_manager.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='시뮬 true / 실차 false'),
        Node(package='autorace_mission', executable='mission_manager', name='mission_manager', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
