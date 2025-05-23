from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Central Control Node
        Node(
            package='central_control',
            executable='central_node',
            name='central_control',
            output='screen'
        ),

        # Turntable Node
        Node(
            package='turntable',
            executable='turntable_node',
            name='turntable',
            output='screen'
        ),

        # Robotic Arm Node
        Node(
            package='robot_arm',
            executable='robot_node',
            name='robotic_arm',
            output='screen'
        ),

        # Capture Node
        Node(
            package='capture',
            executable='capture_node',
            name='capture',
            output='screen'
        ),

        # Meshing Node
        Node(
            package='mesh',
            executable='meshing_node',
            name='meshing',
            output='screen'
        ),

        # User Interface Node
        Node(
            package='user_interface',
            executable='ui_node',
            name='user_interface',
            output='screen'
        ),

        # Local website server (Python's HTTP server, as example)
        ExecuteProcess(
            cmd=['python3', 'web_server.py'],
            cwd='/home/dduboi/ros_ws/src/3DScanner/3DScanner/ui',
            output='screen'
        )
    ])