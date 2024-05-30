#sim.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess,
)    
from launch.substitutions import  LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Arguments
    Time = LaunchDescription([SetParameter(name='use_sim_time', value=True)])
    first_arg = DeclareLaunchArgument(name="first_start", default_value="False",
                                      description="Is the decawave ready?")
    decawave_leo_arg = DeclareLaunchArgument(name="decawave_leo", default_value="False",
                                             description="Run the decawave node using the rover launch?")

    # This node runs the decawave beacons
    decawave_node = Node(
        package="real_hardware",
        executable="decawave",
        name="decawave",
        output="screen",
        condition = IfCondition(LaunchConfiguration('decawave_leo')),
        parameters= [{
            'first_start': LaunchConfiguration("first_start"),
        }]
    )

    # This command launches the velocity controller and the connection to the rover
    command_leocontrol = ExecuteProcess(
        cmd=['ros2', 'run', 'real_hardware', 'leo_control'],
        output='screen',
        name='leo_control',
    )

    # This command runs the node to send goals to the rover
    command_system = ExecuteProcess(
        cmd=['ros2', 'run', 'real_hardware', 'system_control_leo'],
        output='screen',
        name='system_control_leo',
    )
    
    # EKF config file path
    config_file_ekf = os.path.join(
        get_package_share_directory('real_hardware'), 'config', 'ekf.yaml'
    )
    
    # EKF node for the rover
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file_ekf],
        remappings=[('/odometry/filtered', '/odom/filtered')]
    )

    return LaunchDescription([
        Time,                         
        first_arg,
        decawave_leo_arg,
        decawave_node,              
        command_leocontrol,
        command_system,
        ekf_filter_node,
        ])