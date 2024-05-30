from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    leo_description_share = get_package_share_directory("leo_description")

    config_leo_velocity_controller = os.path.join(leo_description_share, 'config', 'control_rovers.yaml')
    
    # Description of the rover
    cont = os.path.join(leo_description_share, 'urdf', 'leo_sim1.urdf.xacro')
    robot_desc = xacro.process_file(cont)#, mappings={"robot":'false'}
    xml = robot_desc.toxml()
    content = {'robot_description': xml}

    # Node to publish the state of the rover
    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        #namespace=LaunchConfiguration("robot_ns"),
        parameters=[content],
    )

    # Spawn the leo rover in gazebo
    spawn_robot = Node(
        name="spawn_entity",
        package="gazebo_ros",
        executable="spawn_entity.py",
        #namespace=LaunchConfiguration("robot_ns"),
        arguments=[
            "-topic", 'robot_description',
            "-entity", LaunchConfiguration("model_name"),
            #"-robot_namespace", LaunchConfiguration("robot_ns"),
            '-x', LaunchConfiguration("x_pos"),
            '-y', LaunchConfiguration("y_pos"),
            '-z', LaunchConfiguration("z_pos"),
        ],
    )         

    # Joint state broadcaster controller for the rover
    joint_state_broadcaster_controller = Node(
        name="joint_state_broadcaster_spawner",
        package="controller_manager",
        executable="spawner",
        #namespace="/rover1/",
        arguments=["joint_state_broadcaster"], #, '-n', LaunchConfiguration("robot_ns")
    )

    # Differential drive controller for the rover
    differential_drive_controller = Node(
        name="diff_drive_controller_spawner",
        package="controller_manager",
        executable="spawner",
        #namespace="/rover1/",
        arguments=["leo1_velocity_controller"], #, '-n', LaunchConfiguration("robot_ns")
    )

    # Wait until the spawn is finished to start the controllers
    delay_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_robot,
            on_start=[TimerAction(
                        period = 10.0,
                        actions = [#PushRosNamespace(LaunchConfiguration("robot_ns")),
                                    joint_state_broadcaster_controller,
                                    differential_drive_controller]
            )],
        ),
    )

    Time = LaunchDescription([SetParameter(name='use_sim_time', value=True)])
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=[leo_description_share, "/urdf/leo_sim1.urdf.xacro"],
                description="Absolute path to robot urdf.xacro file",
            ),
            DeclareLaunchArgument(
                name="fixed",
                default_value="false",
                description='Set to "true" to spawn the robot fixed to the world',
            ),
            DeclareLaunchArgument(
                name="robot_ns", default_value="/", description="Namespace of the robot"
            ),
            DeclareLaunchArgument(
                name="model_name",
                default_value="leo",
                description="The name of the spawned model in Gazebo",
            ),
            DeclareLaunchArgument(
                name="x_pos",
                default_value="0",
                description="Initial x position of the spawned model in Gazebo",
            ),
            DeclareLaunchArgument(
                name="y_pos",
                default_value="0",
                description="Initial y position of the spawned model in Gazebo",
            ),
            DeclareLaunchArgument(
                name="z_pos",
                default_value="0",
                description="Initial z position the spawned model in Gazebo",
            ),
            
            GroupAction(
                actions = [
                    delay_controllers,
                    #PushRosNamespace(LaunchConfiguration("robot_ns")),
                    Time,
                    robot_state_publisher_node,
                    spawn_robot,
                ]
            ),
        ]
    )