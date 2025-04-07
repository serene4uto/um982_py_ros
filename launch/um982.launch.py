from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


ARGS = [
    
]


def generate_launch_description():
    
    config_path = PathJoinSubstitution(
        [FindPackageShare('um982_py_ros'), 'config', 'um982.yaml']
    )
    return LaunchDescription(ARGS + [
        Node(
            package='um982_py_ros',
            executable='um982_node',
            name='um982_node',
            output='screen',
            parameters=[config_path],
            remappings=[
                ('/rtcm', '/um982/rtcm'),
                ('/nmea', '/um982/nmea'),
                ('/fix', '/um982/fix'),
                ('/heading', '/um982/heading')
            ],
        )
    ])