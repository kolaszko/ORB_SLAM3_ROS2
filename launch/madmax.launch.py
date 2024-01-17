from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindPackageShare
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vocab = LaunchConfiguration('vocabulary', default=PathJoinSubstitution([FindPackageShare('orbslam3'), 'vocabulary', 'ORBvoc.txt']))
    config = LaunchConfiguration('config', default=PathJoinSubstitution([FindPackageShare('orbslam3'), 'config', 'stereo', 'madmax_C.yaml']))
    visualize = LaunchConfiguration('visualize', default=False)

    entities = [
        DeclareLaunchArgument(
            'config',
            default_value=config,
            description='Configuration file',
        ),
        DeclareLaunchArgument(
            'vocabulary',
            default_value=vocab,
            description='Vocabulary',
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value=visualize,
            description='Visualization with Pangolin',
        ),

        Node(
            package='orbslam3',
            executable='stereo',
            name='orbslam3',
            arguments=[vocab, config, visualize],
            output='screen',
        )
    ]

    return LaunchDescription(entities)