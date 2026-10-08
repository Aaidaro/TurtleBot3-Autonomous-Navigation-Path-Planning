import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SPAWN_LAUNCH = {
    'local_minima': 'spawn_local_minima.launch.py',
    'narrow_passage': 'spawn_narrow_passage.launch.py',
    'sharp_maze': 'spawn_sharp_maze.launch.py',
}


def launch_setup(context, *args, **kwargs):
    pkg_project = get_package_share_directory('tb3_path_planning')
    environment = LaunchConfiguration('environment').perform(context)

    if environment not in SPAWN_LAUNCH:
        raise RuntimeError(
            f"Unknown environment '{environment}'. Use one of: {', '.join(SPAWN_LAUNCH.keys())}"
        )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', SPAWN_LAUNCH[environment])
        )
    )

    algorithm = LaunchConfiguration('algorithm').perform(context).lower().strip()

    if algorithm in ('apf', 'reactive_apf', 'artificial_potential_field'):
        controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_project, 'launch', 'apf.launch.py')),
            launch_arguments={
                'environment': LaunchConfiguration('environment'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'goal_frame': LaunchConfiguration('goal_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'attractive_kx': LaunchConfiguration('attractive_kx'),
                'attractive_ky': LaunchConfiguration('attractive_ky'),
                'repulsive_alpha': LaunchConfiguration('repulsive_alpha'),
                'repulsive_sigma_x': LaunchConfiguration('repulsive_sigma_x'),
                'repulsive_sigma_y': LaunchConfiguration('repulsive_sigma_y'),
                'repulsive_influence_radius': LaunchConfiguration('repulsive_influence_radius'),
                'max_v': LaunchConfiguration('max_v'),
                'max_w': LaunchConfiguration('max_w'),
            }.items(),
        )
    else:
        controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_project, 'launch', 'planner.launch.py')),
            launch_arguments={
                'environment': LaunchConfiguration('environment'),
                'algorithm': LaunchConfiguration('algorithm'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'inflation_radius': LaunchConfiguration('inflation_radius'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'tracking_waypoint_spacing': LaunchConfiguration('tracking_waypoint_spacing'),
            }.items(),
        )

    return [spawn, controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='local_minima'),
        DeclareLaunchArgument('algorithm', default_value='astar'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('inflation_radius', default_value='0.12'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.20'),
        DeclareLaunchArgument('tracking_waypoint_spacing', default_value='0.05'),
        DeclareLaunchArgument('goal_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('goal_x', default_value='nan'),
        DeclareLaunchArgument('goal_y', default_value='nan'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.12'),
        DeclareLaunchArgument('attractive_kx', default_value='1.20'),
        DeclareLaunchArgument('attractive_ky', default_value='1.20'),
        DeclareLaunchArgument('repulsive_alpha', default_value='0.65'),
        DeclareLaunchArgument('repulsive_sigma_x', default_value='0.35'),
        DeclareLaunchArgument('repulsive_sigma_y', default_value='0.25'),
        DeclareLaunchArgument('repulsive_influence_radius', default_value='0.90'),
        DeclareLaunchArgument('max_v', default_value='0.20'),
        DeclareLaunchArgument('max_w', default_value='1.80'),
        OpaqueFunction(function=launch_setup),
    ])
