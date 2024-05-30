#sim.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import (
    IncludeLaunchDescription,  
    TimerAction,
    ExecuteProcess,
)    
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    Time = LaunchDescription([SetParameter(name='use_sim_time', value=True)])

    package_description = 'tello_gazebo'
    pkg_description = get_package_share_directory(package_description)
    gazebo_models_path = os.path.join(pkg_description, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    package_description_rviz = 'spawner_robots'
    pkg_description_rviz = get_package_share_directory(package_description_rviz)
    rviz_config_file = os.path.join(pkg_description_rviz, 'rviz', 'rviz.rviz')


    # Run sim launch - 1 drone - not in use
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tello_gazebo'), 'launch'),
            '/simple_launch.py']),
    )

    # Run teleop nodes - 1 drone - not in use
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tello_driver'), 'launch'),
            '/teleop_launch.py']),
    )

    # Run sim launch - multiple drone
    simulation_multiple_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tello_gazebo'), 'launch'),
            '/multiple_drones.py']),
    )

    # Spawns just 1 rover in gazebo
    leo_unique = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('spawner_robots'), 'launch'),
            '/spawn_robot_unique.launch.py']),
            launch_arguments={
            'robot_ns': 'rover1',
            'model_name': 'leo1',
            'x_pos': '0.5',
            'y_pos': '1',
            'z_pos': '0',
            #'name_robot_description': 'rover2/robot_description',
        }.items(),
    )

    # Runs a node that offers a takeoff service for multiple drones
    takeoff =  Node(
        package='spawner_robots',
        executable='takeoff_node',
        name='takeoff_node',
        output='screen',
    )

    # Runs a node that publishes the position of the entities in gazebo
    location_node =  Node(
        package='spawner_robots',
        executable='localization_node',
        name='localization_node',
        output='screen',
    )

    # statis TF - not in use
    static_TF = Node(package = "tf2_ros", 
                    executable = "static_transform_publisher",
                    arguments = ["0", "-1", "0", "0", "0", "0", "map", "rover1/base_footprint"])
    
    # Runs rviz - not in use
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Gazebo client - not in use
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',

    )
    
    # Gazebo gui - not in use
    gazebo_gui = ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node

        ], 
        output='screen'
    )
    
    # Run sim launch - 1 rover
    leo1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('spawner_robots'), 'launch'),
            '/spawn_robot.launch.py']),
            launch_arguments={
            'robot_ns': 'rover1',
            'model_name': 'leo1',
            'x_pos': '0',
            'y_pos': '0',
            'z_pos': '0',
            #'name_robot_description': 'rover1/robot_description',
        }.items(),
    )

    # Launches the spawn of the rover
    leo2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('spawner_robots'), 'launch'),
            '/spawn_robot_2.launch.py']),
            launch_arguments={
            'robot_ns': 'rover2',
            'model_name': 'leo2',
            'x_pos': '1',
            'y_pos': '0',
            'z_pos': '0',
            #'name_robot_description': 'rover2/robot_description',
        }.items(),
    )

    delay_leo2 = TimerAction(
                        period = 10.0,
                        actions = [#PushRosNamespace(LaunchConfiguration("robot_ns")),
                                    leo2_launch]
        )

    return LaunchDescription([
        Time,                           # to use sim time
        #simulation_launch,             # 1 drone
        #teleop_launch,                 # teleop with joystick 1 drone
        simulation_multiple_launch,     # spawn multiple drones, enables control using topics
        leo_unique,                     # spawn 1 leo, enables control using topics
        takeoff,                        # control all drones takeof and land
        location_node,
        #rviz_node,                     # to open rviz config file

        #static_TF,                     # static tf to link map frame with leo base
        #gazebo_gui,                    # other things for a double leo spawn, WIP
        #gazebo_client,
        #leo1_launch,
        #leo2_launch,
        #delay_leo2,
    ])