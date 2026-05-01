"""
Panthera G1 SLAM + Nav2 launch file.

Adapted from openmindagi/om1_ros2_sdk's go2_sdk/launch/slam_launch.py
(MIT-licensed). The OM1 v1.0.1 image ships an empty g1_sdk/launch
directory, so this file fills the gap.

Differences from go2:
  - Loads slam_params.yaml and nav2_params.yaml from g1_sdk
  - Hardware-only nodes (rplidar, cmd_vel_to_g1, g1_loco_action) use
    g1_sdk executables instead of go2_sdk's. They are gated by
    UnlessCondition(use_sim) so they don't run in simulation.
  - URDF path points to g1_sdk
  - waypoint_manager is generic and present in both packages, so we
    keep it as g1_sdk's
  - SIM_PARAM_OVERRIDES are kept identical for now. G1 walks slower
    than Go2; if Nav2 controller times out following plans, we'll
    tune these down (vx_max ~0.3 instead of 0.5, etc.).

Usage (from inside the nav container):
  ros2 launch /panthera_launch/panthera_g1_slam.launch.py use_sim:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Identical to Go2 — start here and tune down if humanoid is too slow.
SIM_PARAM_OVERRIDES = {
    "controller_server": {
        "FollowPath.vx_std": 0.2,
        "FollowPath.wz_std": 0.4,
        "FollowPath.vx_max": 0.5,
        "FollowPath.vx_min": -0.35,
        "FollowPath.wz_max": 1.2,
        "FollowPath.ObstaclesCritic.cost_weight": 10.0,
        "FollowPath.ObstaclesCritic.collision_margin_distance": 0.05,
        "FollowPath.ObstaclesCritic.near_goal_distance": 0.4,
        "FollowPath.GoalAngleCritic.cost_power": 4,
        "FollowPath.PreferForwardCritic.cost_weight": 16.0,
        "inflation_layer.cost_scaling_factor": 5.0,
        "inflation_layer.inflation_radius": 0.30,
    },
    "planner_server": {
        "inflation_layer.cost_scaling_factor": 6.0,
        "inflation_layer.inflation_radius": 0.32,
    },
    "velocity_smoother": {
        "max_velocity": [0.5, 0.0, 1.2],
        "min_velocity": [-0.35, 0.0, -1.2],
        "max_accel": [2.0, 0.0, 2.5],
        "max_decel": [-2.0, 0.0, -2.5],
    },
}


def get_node_params(node_name: str, base_config: str, use_sim) -> list:
    """Merge base config with simulation overrides if needed."""
    params = [base_config, {"use_sim_time": use_sim}]
    if node_name in SIM_PARAM_OVERRIDES:
        params.append(SIM_PARAM_OVERRIDES[node_name])
    return params


def generate_launch_description():
    pkg_dir = get_package_share_directory("g1_sdk")

    # G1 URDF (used by robot_state_publisher when not in sim mode)
    urdf_file = os.path.join(pkg_dir, "urdf", "g1.urdf")
    robot_desc = ""
    if os.path.exists(urdf_file):
        with open(urdf_file, "r") as infp:
            robot_desc = infp.read()

    slam_config_file = os.path.join(pkg_dir, "config", "slam_params.yaml")
    nav2_config_file = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    m_explorer_config_file = os.path.join(
        pkg_dir, "config", "m_explorer_ros2_params.yaml"
    )

    use_sim = LaunchConfiguration("use_sim", default="false")
    map_yaml_file = LaunchConfiguration("map_yaml_file", default="")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value=use_sim,
                description="Whether to use simulation"),
            DeclareLaunchArgument("map_yaml_file", default_value=map_yaml_file,
                description="Full path to map yaml (empty for SLAM mode)"),

            # Hardware nodes — skipped in sim mode -----------------------------
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="g1_sdk",
                executable="g1_jointstate",
                name="g1_jointstate",
                output="screen",
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="g1_sdk",
                executable="g1_odom",
                name="g1_odom",
                output="screen",
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="g1_sdk",
                executable="g1_scan_relay",
                name="g1_scan_relay",
                output="screen",
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="g1_sdk",
                executable="cmd_vel_to_g1",
                name="cmd_vel_to_g1",
                output="screen",
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="g1_sdk",
                executable="g1_loco_action",
                name="g1_loco_action",
                output="screen",
                condition=UnlessCondition(use_sim),
            ),
            Node(
                package="g1_sdk",
                executable="waypoint_manager",
                name="waypoint_manager",
                output="screen",
            ),

            # SLAM ------------------------------------------------------------
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[slam_config_file, {"use_sim_time": use_sim}],
                output="screen",
            ),

            # Nav2 lifecycle manager ------------------------------------------
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim},
                    {"autostart": True},
                    {"node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ]},
                ],
            ),

            # Nav2 stack ------------------------------------------------------
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=get_node_params("controller_server", nav2_config_file, use_sim),
                remappings=[("/cmd_vel", "/cmd_vel")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[nav2_config_file, {"use_sim_time": use_sim}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=get_node_params("planner_server", nav2_config_file, use_sim),
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_config_file, {"use_sim_time": use_sim}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_config_file, {"use_sim_time": use_sim}],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[nav2_config_file, {"use_sim_time": use_sim}],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=get_node_params("velocity_smoother", nav2_config_file, use_sim),
                remappings=[
                    ("/cmd_vel", "/cmd_vel_nav"),
                    ("/cmd_vel_smoothed", "/cmd_vel"),
                ],
            ),

            # Frontier exploration --------------------------------------------
            Node(
                package="frontier_explorer",
                executable="explore",
                name="frontier_explorer",
                output="screen",
                parameters=[m_explorer_config_file, {"use_sim_time": use_sim}],
            ),
        ]
    )
