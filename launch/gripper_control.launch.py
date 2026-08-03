"""
Launch file for gripper and sensor nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_magpie_control = get_package_share_directory('magpie_control')
    
    # Declare arguments
    gripper_config_file = DeclareLaunchArgument(
        'gripper_config',
        default_value=os.path.join(pkg_magpie_control, 'config', 'gripper_config.yaml'),
        description='Path to gripper configuration file'
    )
    
    # Gripper Node
    gripper_node = Node(
        package='magpie_control',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
        parameters=[LaunchConfiguration('gripper_config')],
    )
    
    # F/T Sensor Node
    ft_sensor_node = Node(
        package='magpie_control',
        executable='ft_sensor_node',
        name='ft_sensor_node',
        output='screen',
        parameters=[LaunchConfiguration('gripper_config')],
    )
    
    # Tactile Sensor Node (optional)
    tactile_sensor_node = Node(
        package='magpie_control',
        executable='tactile_sensor_node',
        name='tactile_sensor_node',
        output='screen',
        parameters=[LaunchConfiguration('gripper_config')],
    )
    
    # UR5 Node
    ur5_node = Node(
        package='magpie_control',
        executable='ur5_node',
        name='ur5_node',
        output='screen',
        parameters=[LaunchConfiguration('gripper_config')],
    )

    # DeliGrasp Node — expects camera on /camera/gripper_camera namespace
    # Launch realsense with: ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera
    deligrasp_node = Node(
        package='magpie_control',
        executable='deligrasp_node',
        name='deligrasp_node',
        output='screen',
        parameters=[LaunchConfiguration('gripper_config')],
    )

    return LaunchDescription([
        gripper_config_file,
        gripper_node,
        ft_sensor_node,
        tactile_sensor_node,
        ur5_node,
        deligrasp_node,
    ])
