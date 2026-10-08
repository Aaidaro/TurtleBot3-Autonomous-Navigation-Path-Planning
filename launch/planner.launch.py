import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


START_GOAL = {
    'local_minima': ((0.0, 0.0), (4.0, 0.0), 'local_minima_map.yaml'),
    'narrow_passage': ((0.0, 0.0), (4.0, 0.0), 'narrow_passage_map.yaml'),
    'sharp_maze': ((-4.5, -4.5), (4.5, 4.5), 'sharp_maze_map.yaml'),
}


def launch_setup(context, *args, **kwargs):
    pkg_project = get_package_share_directory('tb3_path_planning')
    environment = LaunchConfiguration('environment').perform(context)
    algorithm = LaunchConfiguration('algorithm').perform(context)
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    inflation_radius = ParameterValue(LaunchConfiguration('inflation_radius'), value_type=float)
    lookahead_distance = ParameterValue(LaunchConfiguration('lookahead_distance'), value_type=float)
    tracking_waypoint_spacing = ParameterValue(LaunchConfiguration('tracking_waypoint_spacing'), value_type=float)

    if environment not in START_GOAL:
        raise RuntimeError(
            f"Unknown environment '{environment}'. Use one of: {', '.join(START_GOAL.keys())}"
        )

    start, goal, map_yaml = START_GOAL[environment]
    map_file = os.path.join(pkg_project, 'maps', map_yaml)

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    planner = Node(
        package='tb3_path_planning',
        executable='global_planner',
        name='tb3_global_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'algorithm': algorithm,
            'environment': environment,
            'start_x': start[0],
            'start_y': start[1],
            'goal_x': goal[0],
            'goal_y': goal[1],
            'inflation_radius': inflation_radius,
            'tracking_waypoint_spacing': tracking_waypoint_spacing,
        }],
    )

    pure_pursuit = Node(
        package='tb3_path_planning',
        executable='pure_pursuit_tracker',
        name='pure_pursuit_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'lookahead_distance': lookahead_distance,
        }],
    )

    return [map_server, lifecycle_manager, planner, pure_pursuit]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'environment',
            default_value='local_minima',
            description='local_minima, narrow_passage, or sharp_maze',
        ),
        DeclareLaunchArgument(
            'algorithm',
            default_value='astar',
            description='astar or rrt_star',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time',
        ),
        DeclareLaunchArgument(
            'inflation_radius',
            default_value='0.12',
            description='Obstacle inflation radius in meters',
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='0.20',
            description='Pure Pursuit lookahead distance in meters',
        ),
        DeclareLaunchArgument(
            'tracking_waypoint_spacing',
            default_value='0.05',
            description='Maximum spacing in meters between waypoints published to the tracker',
        ),
        OpaqueFunction(function=launch_setup),
    ])
