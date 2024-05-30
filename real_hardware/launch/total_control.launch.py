#sim.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import (
    IncludeLaunchDescription,  
    DeclareLaunchArgument, 
)    
from launch.substitutions import  LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # Arguments
    Time = LaunchDescription([SetParameter(name='use_sim_time', value=True)])
    first_arg = DeclareLaunchArgument(name="first_start", default_value="False", 
                                      description="Is the decawave ready?")
    decawave_leo_arg = DeclareLaunchArgument(name="decawave_leo", default_value="False",
                                             description="Run the decawave node using the rover launch?")
    decawave_tello_arg = DeclareLaunchArgument(name="decawave_tello", default_value="False",
                                               description="Run the decawave node using the rover launch?")
    path_visual_arg = DeclareLaunchArgument(name="path_visual", default_value="True", 
                                            description="Mandatory to visualize the path comparative in rviz")
    rviz_arg = DeclareLaunchArgument(name="rviz", default_value="True", 
                                            description="Open rviz?")
    rosbag_arg = DeclareLaunchArgument(name="rosbag", default_value="False", 
                                            description="Creates a rosbag named data (overwrites any other folder)")
    complementary_arg = DeclareLaunchArgument(name="complementary", default_value="True", 
                                            description="To enable 'path_visual', 'rviz' and 'rosbag' args")
    leo_arg = DeclareLaunchArgument(name="leo", default_value="True", 
                                            description="Launch the nodes for the leo?")
    tello_arg = DeclareLaunchArgument(name="tello", default_value="False", 
                                            description="Launch the nodes for the tello?")
    
    # This node runs the decawave beacons
    decawave_node = Node(
        package="real_hardware",
        executable="decawave",
        name="decawave",
        output="screen",
        parameters= [{
            'first_start': LaunchConfiguration("first_start"),
        }]
    )

    # Launches the rover nodes
    leo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('real_hardware'), 'launch'),
            '/rover_control.launch.py']),
            launch_arguments={
                'decawave': LaunchConfiguration('decawave_leo')
            }.items(),
        condition = IfCondition(LaunchConfiguration('leo')),
    )

    # Launches the drone nodes
    tello_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('real_hardware'), 'launch'),
            '/tello_control.launch.py']),
            launch_arguments={
                'decawave': LaunchConfiguration('decawave_tello')
            }.items(),
        condition = IfCondition(LaunchConfiguration('tello')),
    )
    
    # Launches the complementary nodes
    complementary_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('real_hardware'), 'launch'),
            '/complementary.launch.py']),
            launch_arguments={
                'path_visual': LaunchConfiguration('path_visual'),
                'rviz': LaunchConfiguration('rviz'),
                'rosbag': LaunchConfiguration('rosbag')
            }.items(),
        condition = IfCondition(LaunchConfiguration('complementary')),
    )

    return LaunchDescription([
        Time,                  
        first_arg,
        decawave_leo_arg,
        decawave_tello_arg,
        path_visual_arg,
        rviz_arg,
        rosbag_arg,
        complementary_arg,
        leo_arg,
        tello_arg,
        
        decawave_node,                
        leo_launch,
        tello_launch,
        complementary_launch
        ])