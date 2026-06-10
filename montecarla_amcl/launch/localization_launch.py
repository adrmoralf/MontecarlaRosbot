"""Lanza map_server + montecarla_amcl + lifecycle_manager."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    map_yaml     = LaunchConfiguration('map')
    autostart    = LaunchConfiguration('autostart')
    wifi_enabled = LaunchConfiguration('wifi_enabled')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file',  default_value='/params.yaml'),
        DeclareLaunchArgument('map',          default_value=''),
        DeclareLaunchArgument('autostart',    default_value='true'),
        DeclareLaunchArgument('wifi_enabled', default_value='false'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'yaml_filename': map_yaml,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),

        Node(
            package='montecarla_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                params_file,
                {
                    # Convertir string 'true'/'false' a bool Python
                    'wifi_enabled': PythonExpression(
                        ["'", wifi_enabled, "'.lower() == 'true'"]),
                    'use_sim_time': use_sim_time,
                },
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': ['map_server', 'amcl'],
                },
            ],
        ),
    ])
