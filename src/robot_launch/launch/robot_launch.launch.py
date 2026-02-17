import xacro
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    descr_dir = get_package_share_directory('robot_description')
    
    xacro_file = os.path.join(descr_dir, 'urdf', 'robot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file).toxml()
    

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
    )
    
    robot_logic_node=Node(
        package="robot_logic",
        executable="StateMachineNode",

    )
    robot_motors_node=Node(
        package="robot_motors",
        executable="Motor_node",
    )

    robot_sensors_node=Node(
        package="robot_sensors",
        executable="distance_sensor",
    )

    ld.add_action(robot_logic_node)
    ld.add_action(robot_motors_node)
    ld.add_action(robot_sensors_node)
    ld.add_action(robot_state_publisher)
    return ld
