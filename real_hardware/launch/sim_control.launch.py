#sim.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import (
    IncludeLaunchDescription,  
    TimerAction,
)    
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    Time = LaunchDescription([SetParameter(name='use_sim_time', value=True)])
    
    # Launches the simulation on gazebo
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('move_robots'), 'launch'),
            '/move.launch.py']),
    )
    
    # Runs the simulation node
    simulation = Node(
        package="real_hardware",
        executable="simulation",
        name="simulation",
        output="screen",
    )
    
    delay_simulation = TimerAction(
        period=10.0,
        actions=[simulation]
    )
    
    return LaunchDescription([
        Time,                           # to use sim time
        sim_launch,
        delay_simulation
        ])