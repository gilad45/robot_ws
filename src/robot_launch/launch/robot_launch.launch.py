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

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld19',
        arguments=['0','0','0.18','0','0','0','base_link','base_laser']
    )

    ros2_laser_scan_matcher = Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            output='screen',
            parameters=[{
                'publish_odom': '/odom',
                'publish_tf': True,
                # You can add other parameters here, like frames:
                # 'base_frame': 'base_link',
                # 'map_frame': 'map',
            }]
    )

    ld.add_action(ros2_laser_scan_matcher)
    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser_tf_node)
    ld.add_action(robot_logic_node)
    ld.add_action(robot_motors_node)
    ld.add_action(robot_sensors_node)
    ld.add_action(robot_state_publisher)
    return ld




    