from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import re


def generate_launch_description():
    ld = LaunchDescription()

    # Get the installed URDF path
    pkg_path = get_package_share_directory('kinematics_so100')
    urdf_path = os.path.join(pkg_path, 'so100_description', 'so100.urdf')
    
    print(f"[display.launch] Reading URDF from: {urdf_path}")
    
    # Read URDF
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    # Check for relative paths before replacement
    if 'filename="assets/' in robot_desc:
        print("[display.launch] WARNING: Found relative paths in URDF, fixing...")
        # Fix any relative paths in the URDF to package:// format
        robot_desc = re.sub(
            r'filename="assets/',
            'filename="package://kinematics_so100/so100_description/assets/',
            robot_desc
        )
    else:
        print("[display.launch] No relative paths found in URDF")
    
    # Verify replacement
    first_mesh = [line for line in robot_desc.split('\n') if 'filename=' in line and 'mesh' in line]
    if first_mesh:
        print(f"[display.launch] Sample mesh path after fix: {first_mesh[0].strip()}")

    # GUI argument
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    ld.add_action(gui_arg)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 50.0,
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # Joint State Publisher GUI (only when gui:=true)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    ld.add_action(joint_state_publisher_gui_node)

    # Note: When gui:=false, no joint_state_publisher is launched
    # External topics should publish to /joint_states directly

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('kinematics_so100'),
        'rviz',
        'display.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    ld.add_action(rviz_node)

    return ld