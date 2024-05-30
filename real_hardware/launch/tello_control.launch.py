#sim.launch.py
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
    Time = LaunchDescription([SetParameter(name='use_sim_time', value=True)])
    first_arg = DeclareLaunchArgument(name="first_start", default_value="False",
                                      description="Is the decawave ready?")
    decawave_tello_arg = DeclareLaunchArgument(name="decawave_tello", default_value="True",
                                               description="Run the decawave node using the tello launch?")

    # This node runs the decawave beacons
    decawave_node = Node(
        package="real_hardware",
        executable="decawave",
        name="decawave",
        output="screen",
        condition = IfCondition(LaunchConfiguration('decawave_tello')),
        parameters= [{
            'first_start': LaunchConfiguration("first_start"),
        }]
    )
    # This command launches the velocity controller and the goal controller to the drone
    command_system_tello = ExecuteProcess(
        cmd=['ros2', 'run', 'real_hardware', 'system_control_tello'],
        output='screen',
        name='system_control_tello',
    )

    return LaunchDescription([
        Time,                      
        first_arg,
        decawave_tello_arg,
        decawave_node,               
        command_system_tello,           
        ])