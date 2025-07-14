import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('moi_exp_lite')
    cam_share = get_package_share_directory('turtlebot3_autorace_camera')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths to calibration launch files
    intrinsic_launch = PythonLaunchDescriptionSource(
        os.path.join(cam_share, 'launch', 'intrinsic_camera_calibration.launch.py')
    )
    extrinsic_launch = PythonLaunchDescriptionSource(
        os.path.join(cam_share, 'launch', 'extrinsic_camera_calibration.launch.py')
    )

    # Paths to our package's launch files
    detect_launch = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'detect_object.launch.py')
    )
    explore_launch = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'explore.launch.py')
    )

    # Params files
    controller_params_file = os.path.join(pkg_share, 'param', 'explore_controller_params.yaml')
    explorer_params_file = os.path.join(pkg_share, 'param', 'explorer_params.yaml')
    detected_yaml = os.path.join(pkg_share, 'detected_objects.yaml')

    # Base directory for map saving
    map_dir = os.path.join(pkg_share, 'map')
    os.makedirs(map_dir, exist_ok=True)

    # Helper to build map save command with logging
    map_saver_cmd = (
        'while true; do '
        'filename="lab_$(date +\"%Y%m%d_%H%M%S\")"; '
        'ros2 run nav2_map_server map_saver_cli -f ' + map_dir + '/$filename && '
        'echo "[INFO] [map_saver]: Saved map $filename to ' + map_dir + '"; '
        # Just keep last 10 saves
        'ls -1tr ' + map_dir + '/lab_* | head -n -10 | xargs -r rm --; '
        'sleep 10; '
        'done'
    )

    return LaunchDescription([
        declare_use_sim_time,
        # t=0s: Intrinsic camera calibration
        TimerAction(
            period=0.0,
            actions=[IncludeLaunchDescription(intrinsic_launch)]
        ),

        # t=7s: Extrinsic camera calibration
        TimerAction(
            period=7.0,
            actions=[IncludeLaunchDescription(extrinsic_launch)]
        ),

        # t=12s: Color detection node
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='image_transport',
                    executable='republish',
                    name='republish',
                    arguments=[
                        'raw', 'compressed',
                        '--ros-args',
                        '-r', 'in:=/camera/image_raw',
                        '-r', 'out/compressed:=/camera/image_raw/compressed'
                    ],
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
                IncludeLaunchDescription(
                    detect_launch,
                    launch_arguments={'calibration_mode': 'True',
                                      'use_sim_time':use_sim_time}.items()
                )
            ]
        ),

        # t=16s: Exploration and controller
        TimerAction(
            period=16.0,
            actions=[
                IncludeLaunchDescription(
                    explore_launch,
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': explorer_params_file,
                    }.items()
                ),
                Node(
                    package='moi_exp_lite',
                    executable='explore_controller',
                    name='explore_controller',
                    output='screen',
                    parameters=[
                        controller_params_file,
                        {
                            'use_sim_time': use_sim_time,
                            'yaml_file_path': detected_yaml,
                        },
                    ],
                ),
            ]
        ),

        # Start periodic map saving 10s after launch and then every 10s
        TimerAction(
            period=26.0,  # initial delay (sum of prior delays) + 10s buffer
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-lc', map_saver_cmd],
                    output='screen'
                )
            ]
        ),
    ])

