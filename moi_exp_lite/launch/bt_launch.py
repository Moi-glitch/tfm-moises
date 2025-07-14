import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

GROOT2_EXECUTABLE = os.environ.get('GROOT2_EXECUTABLE', 'groot2')
groot2_executable = GROOT2_EXECUTABLE

def get_bt_and_visualization_nodes(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = get_package_share_directory('moi_exp_lite')
    bt_xml = join(pkg_share, 'bt_xml', 'bt.xml')
    xml_file_path = bt_xml
    bt_node = Node(
        package='moi_exp_lite',
        executable='bt_node',
        name='bt_node',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'bt_xml': bt_xml
        }]
    )
    return [
        bt_node,
        ExecuteProcess(
            cmd=[groot2_executable, "--nosplash", "true", "--file", xml_file_path]
        ),
    ]

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    return LaunchDescription([use_sim_time_arg, OpaqueFunction(function=get_bt_and_visualization_nodes)])
