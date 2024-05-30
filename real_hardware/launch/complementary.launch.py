#sim.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ( 
    DeclareLaunchArgument, 
    RegisterEventHandler, 
    ExecuteProcess,
)    
from launch.substitutions import  LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Arguments
    path_visual_arg = DeclareLaunchArgument(name="path_visual", default_value="True", 
                                            description="Mandatory to visualize the path comparative in rviz")
    rviz_arg = DeclareLaunchArgument(name="rviz", default_value="True", 
                                            description="Open rviz?")
    rosbag_arg = DeclareLaunchArgument(name="rosbag", default_value="False", 
                                            description="Creates a rosbag named data (overwrites any other folder)")
    
    # This node prepares the data for visualization in rviz
    demo_node = Node(
        package="real_hardware",
        executable="demo",
        output="screen",
        condition = IfCondition(LaunchConfiguration('path_visual')),
    )
    
    # Config file path for rviz
    config_file_rviz = os.path.join(
        get_package_share_directory('real_hardware'), 'rviz', 'config1.rviz'
    )
    
    # rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', config_file_rviz],
        condition = IfCondition(LaunchConfiguration('rviz')),
    )
    
    # This command deletes the folder data to make space for the rosbag
    command_delete_data = ExecuteProcess(
        cmd=['rm', '-r', 'data/'],
        output='screen',
        name='system_control_leo',
        condition = IfCondition(LaunchConfiguration('rosbag')),
    )
    
    # Tis command runs the rosbag and saves it in a folder called data
    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--all', '-o', 'data/'],
        output='screen',
        name='system_control_leo',
    )
    
    command_rosbag = RegisterEventHandler(
        OnProcessExit(
        target_action= command_delete_data,
        on_exit=[rosbag],
        )
    )
    
    return LaunchDescription([
        path_visual_arg,
        rviz_arg,
        rosbag_arg,
        demo_node,
        rviz,
        command_delete_data,
        command_rosbag,
        ])