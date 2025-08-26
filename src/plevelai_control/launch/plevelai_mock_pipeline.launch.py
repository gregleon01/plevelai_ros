from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='plevelai_vision', executable='mock_camera', name='mock_camera', output='screen'),
        Node(package='plevelai_vision', executable='detector_node', name='detector', output='screen'),
        Node(package='plevelai_control', executable='targeting_node', name='targeting', output='screen'),
        Node(package='plevelai_control', executable='firing_node', name='firing', output='screen'),
        Node(package='plevelai_control', executable='mock_laser', name='mock_laser', output='screen'),
    ])
