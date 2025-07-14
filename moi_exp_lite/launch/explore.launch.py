import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths and configurations
    default_config = os.path.join(
        get_package_share_directory("moi_exp_lite"),
        "param",
        "explorer_params.yaml"
    )

    # Launch arguments
    declare_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description="Path to the exploration params.yaml"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the explore node'
    )

    # Substitutions
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Remappings for tf
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # Node
    explore_node = Node(
        package="moi_exp_lite",
        executable="explore_node",
        name='explore_node',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_arg)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_namespace)
    ld.add_action(explore_node)

    return ld

