from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vocab = LaunchConfiguration('vocabulary', default=PathJoinSubstitution(
        [FindPackageShare('orbslam3'), 'vocabulary', 'ORBvoc.txt']))
    config = LaunchConfiguration('config', default=PathJoinSubstitution(
        [FindPackageShare('orbslam3'), 'config', 'stereo', 'madmax_C_lite.yaml']))
    rectify = LaunchConfiguration('rectify', default='false')

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
            'rectify',
            default_value=rectify,
            description='Rectify',
        ),

        Node(
            package='orbslam3',
            executable='stereo',
            name='orbslam3',
            arguments=[vocab, config, rectify],
            output='screen',
            remappings=[
                ('/camera/left', '/lunarsim/camera_left/raw'),
                ('/camera/right', '/lunarsim/camera_right/raw'),
            ]
        )
    ]

    return LaunchDescription(entities)
