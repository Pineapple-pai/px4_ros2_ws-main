"""Minimal bringup for PX4 local-position control with local goals.

PX4 owns state estimation (including optical-flow fusion). This launch file
does not start mapping, a global planner, LiDAR, FAST-LIO, or Nav2.
Near-field distance topics are supplied by an external perception node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("max_horizontal_velocity_m_s", default_value="0.5"),
        DeclareLaunchArgument("max_vertical_velocity_m_s", default_value="0.3"),
        DeclareLaunchArgument("acceptance_radius_m", default_value="0.25"),
        DeclareLaunchArgument("mission_timeout_s", default_value="60.0"),
        DeclareLaunchArgument("obstacle_data_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("target_point_topic", default_value="/autonomy/target_ned"),
        DeclareLaunchArgument("front_obstacle_distance_topic",
                              default_value="/perception/front_obstacle_distance"),
        DeclareLaunchArgument("left_obstacle_distance_topic",
                              default_value="/perception/left_obstacle_distance"),
        DeclareLaunchArgument("right_obstacle_distance_topic",
                              default_value="/perception/right_obstacle_distance"),
        DeclareLaunchArgument("obstacle_stop_distance_m", default_value="0.8"),
        DeclareLaunchArgument("obstacle_abort_distance_m", default_value="0.35"),
        DeclareLaunchArgument("avoidance_trigger_distance_m", default_value="1.5"),
        DeclareLaunchArgument("avoidance_clearance_m", default_value="0.8"),
        DeclareLaunchArgument("avoidance_lateral_offset_m", default_value="0.7"),
        DeclareLaunchArgument("avoidance_forward_offset_m", default_value="0.8"),
        Node(
            package="px4_autonomy_mode",
            executable="px4_autonomy_mode",
            name="px4_autonomy_mode",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "max_horizontal_velocity_m_s": LaunchConfiguration(
                    "max_horizontal_velocity_m_s"),
                "max_vertical_velocity_m_s": LaunchConfiguration(
                    "max_vertical_velocity_m_s"),
                "acceptance_radius_m": LaunchConfiguration("acceptance_radius_m"),
                "mission_timeout_s": LaunchConfiguration("mission_timeout_s"),
                "obstacle_data_timeout_s": LaunchConfiguration("obstacle_data_timeout_s"),
                "target_point_topic": LaunchConfiguration("target_point_topic"),
                "accept_runtime_target": True,
                "wait_for_runtime_target": True,
                "mission_file": "",
                "mission_waypoints_ned": "",
                "enable_obstacle_hold": True,
                "require_obstacle_data": True,
                "enable_local_avoidance": True,
                "front_obstacle_distance_topic": LaunchConfiguration(
                    "front_obstacle_distance_topic"),
                "left_obstacle_distance_topic": LaunchConfiguration(
                    "left_obstacle_distance_topic"),
                "right_obstacle_distance_topic": LaunchConfiguration(
                    "right_obstacle_distance_topic"),
                "obstacle_stop_distance_m": LaunchConfiguration("obstacle_stop_distance_m"),
                "obstacle_abort_distance_m": LaunchConfiguration("obstacle_abort_distance_m"),
                "avoidance_trigger_distance_m": LaunchConfiguration(
                    "avoidance_trigger_distance_m"),
                "avoidance_clearance_m": LaunchConfiguration("avoidance_clearance_m"),
                "avoidance_lateral_offset_m": LaunchConfiguration(
                    "avoidance_lateral_offset_m"),
                "avoidance_forward_offset_m": LaunchConfiguration(
                    "avoidance_forward_offset_m"),
            }],
        ),
    ])
