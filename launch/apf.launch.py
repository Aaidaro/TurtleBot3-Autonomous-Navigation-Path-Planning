from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# For APF we do not start nav2_map_server. Goals are expressed in odom by
# default, so sharp_maze uses the displacement from spawn (-4.5, -4.5) to
# map goal (4.5, 4.5), i.e. approximately (9.0, 9.0) in odom coordinates.
APF_DEFAULT_GOALS = {
    'local_minima': (4.0, 0.0),
    'narrow_passage': (4.0, 0.0),
    'sharp_maze': (9.0, 9.0),
}


def _float_or_default(raw_value, default_value):
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return default_value
    return default_value if value != value else value  # NaN check


def launch_setup(context, *args, **kwargs):
    environment = LaunchConfiguration('environment').perform(context)
    if environment not in APF_DEFAULT_GOALS:
        options = ', '.join(APF_DEFAULT_GOALS.keys())
        raise RuntimeError(f"Unknown environment '{environment}'. Use one of: {options}")

    default_goal_x, default_goal_y = APF_DEFAULT_GOALS[environment]
    goal_x = _float_or_default(LaunchConfiguration('goal_x').perform(context), default_goal_x)
    goal_y = _float_or_default(LaunchConfiguration('goal_y').perform(context), default_goal_y)

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    apf_controller = Node(
        package='tb3_path_planning',
        executable='apf_reactive_controller',
        name='apf_reactive_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'goal_frame': LaunchConfiguration('goal_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_tolerance': ParameterValue(
                LaunchConfiguration('goal_tolerance'), value_type=float
            ),
            'attractive_kx': ParameterValue(
                LaunchConfiguration('attractive_kx'), value_type=float
            ),
            'attractive_ky': ParameterValue(
                LaunchConfiguration('attractive_ky'), value_type=float
            ),
            'repulsive_alpha': ParameterValue(
                LaunchConfiguration('repulsive_alpha'), value_type=float
            ),
            'repulsive_sigma_x': ParameterValue(
                LaunchConfiguration('repulsive_sigma_x'), value_type=float
            ),
            'repulsive_sigma_y': ParameterValue(
                LaunchConfiguration('repulsive_sigma_y'), value_type=float
            ),
            'repulsive_influence_radius': ParameterValue(
                LaunchConfiguration('repulsive_influence_radius'), value_type=float
            ),
            'max_v': ParameterValue(LaunchConfiguration('max_v'), value_type=float),
            'max_w': ParameterValue(LaunchConfiguration('max_w'), value_type=float),
        }],
    )

    return [apf_controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='local_minima'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
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
