from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='config/default_config.yaml',
        description='Path to YAML configuration file'
    )
    
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='/data/recordings/dataset.lance',
        description='Path to Lance dataset'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically start recording on launch'
    )
    
    # Recorder node
    recorder_node = Node(
        package='axon',
        executable='axon_node',
        name='axon',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'dataset_path': LaunchConfiguration('dataset_path'),
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )
    
    return LaunchDescription([
        config_path_arg,
        dataset_path_arg,
        auto_start_arg,
        recorder_node,
    ])

