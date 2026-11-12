# wego teleop_launch.py 복사본 (원본은 안 건드림, 11-12): EKF (robot_localization) 빼고, 카메라 점구름·컬러 점구름·IR 끔.
# 우리 노드는 /odom·/imu·컬러·깊이 영상만 씀 — /odometry/filtered·TF·점구름·IR 구독 없음 (코드·실행 중 구독자 확인).
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("degree", default_value="0.0"),
        IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare("limo_description"), "launch", "load_urdf.launch.py"])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("wego"), "launch", "camera_tilt_launch.py"])]),
                                 launch_arguments={"degree": LaunchConfiguration("degree")}.items()),
        IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare("limo_base"), "launch", "limo_base.launch.py"])),
        IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare("orbbec_camera"), "launch", "astra_stereo_u3.launch.py"]),
                                 launch_arguments={"enable_point_cloud": "false", "enable_colored_point_cloud": "false",
                                                   "enable_ir": "false"}.items()),
        IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare("ydlidar_ros2_driver"), "launch", "ydlidar.launch.py"])),
    ])
