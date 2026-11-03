from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("voxel_map"))
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    use_imu = ParameterValue(LaunchConfiguration("use_imu"), value_type=bool)
    actions = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("input_topic", default_value="/rslidar_points"),
        DeclareLaunchArgument("imu_topic", default_value="/rslidar_imu_data"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=str(share / "config" / "robosenseairy.yaml")),
        Node(package="voxel_map", executable="voxel_mapping_odom", name="voxel_mapping_odom",
             output="screen", parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time, "imu.imu_en": use_imu}],
             remappings=[("/rslidar_points", LaunchConfiguration("input_topic")),
                         ("/rslidar_imu_data", LaunchConfiguration("imu_topic"))]),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(share.joinpath('rviz_cfg', 'voxel_mapping.rviz'))],
             parameters=[{"use_sim_time": use_sim_time}]),
    ]
    return LaunchDescription(actions)
