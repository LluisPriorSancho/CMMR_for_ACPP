#sim.launch.py
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import (
    IncludeLaunchDescription,  
)    
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    Time = LaunchDescription([SetParameter(name='use_sim_time', value=True)])

    # Run sim launch - 1 drone
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('spawner_robots'), 'launch'),
            '/sim.launch.py']),
    )

    # Run velocity controller for the rover in simulation
    target_robot_node = Node(
        package="move_robots",
        executable="velControlledRobot",
        name="velControlledRobot",
        output="screen",
    )

    # Run velocity controller for the drone in simulation
    target_drone1 = Node(
        package= "move_robots",
        executable="velControlledDrone",
        name="velControlledDrone",
        namespace="drone1",
        output="screen",
        parameters=[{
                'target_pub': '/drone1/cmd_vel',
                'target_drone': 'base_link_1',
                'target_topic': '/drone1/new_goal',
                }]
    )

    target_drone2 = Node(
        package= "move_robots",
        executable="velControlledDrone",
        name="velControlledDrone",
        namespace="drone2",
        output="screen",
        parameters=[{
                'target_pub': '/drone2/cmd_vel',
                'target_drone': 'base_link_2',
                'target_topic': '/drone2/new_goal',
                }]
    )

    target_drone3 = Node(
        package= "move_robots",
        executable="velControlledDrone",
        name="velControlledDrone",
        namespace="drone3",
        output="screen",
        parameters=[{
                'target_pub': '/drone3/cmd_vel',
                'target_drone': 'base_link_3',
                'target_topic': '/drone3/new_goal',
                }]
    )

    target_drone4 = Node(
        package= "move_robots",
        executable="velControlledDrone",
        name="velControlledDrone",
        namespace="drone4",
        output="screen",
        parameters=[{
                'target_pub': '/drone4/cmd_vel',
                'target_drone': 'base_link_4',
                'target_topic': '/drone4/new_goal',
                }]
    )

    # To publish position goals for the drones forming a line once given a goal point
    line = Node(
        package="move_robots",
        executable="line",
        name="line",
        output="screen",
    )

    # To publish position goals for the drones forming a line while following the rover
    follower = Node(
        package="move_robots",
        executable="robot_follower",
        name="robot_follower",
        output="screen",
    )

    return LaunchDescription([
        Time,                           # To use sim time
        simulation_launch,              # To launch simulation
        target_robot_node,              # To move the rover
        target_drone1,                  # To move the drone
        #target_drone2,
        #target_drone3,
        #target_drone4,
        #line,
        #follower,
    ])