# CMMR_for_ACPP
# Cooperative Multi-Modal Robots for Autonomous Coverage Path Planning

This git repository includes all the necessary files to create a Cooperative Multi-Modal Robot for Autonomous Coverage Path Planning in ROS2 - humble. It enables the possibility to work in a virtual environment powered by Gazebo or to work directly in the real hardware using a DJI Tello drone and a Leo rover.

This repository is part of the Master's thesis "Cooperative Multi-Modal Robots for Autonomous Coverage Path Planning" by Lluis Prior. For further information access [here](https://drive.google.com/file/d/1O6j72nzqwbA8HTsFL853CV2fNH2on2dY/view?usp=sharing)

[![Video](https://res.cloudinary.com/marcomontalbano/image/upload/v1739005576/video_to_markdown/images/google-drive--1eD390trXYyCGPKiSVs8rt14_KYjpfMv8-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://drive.google.com/file/d/1eD390trXYyCGPKiSVs8rt14_KYjpfMv8/view?usp=drive_link "Video")
______________________________________________________________________

<details open>
<summary><b>Dependences:</b></summary> 

-   [robot_localization](https://github.com/automaticaddison/robot_localization)
-	[ros2_control](https://github.com/ros-controls/ros2_control)
-   [xacro](https://github.com/ros/xacro)
-   [rosidl](https://github.com/ros2/rosidl)
-   [gazebosim](http://classic.gazebosim.org/)
-   [gazebo_ros_pks](https://github.com/ros-simulation/gazebo_ros_pkgs)
-   [rqt-release](https://github.com/ros2-gbp/rqt-release)

```
cd to_existing_workspace/src/
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-control*
sudo apt install ros-humble-xacro
sudo apt install ros-humble-rosidl-*
sudo apt install gazebo*
sudo apt install ros-humble-rqt*
sudo apt install ros-humble-gazebo-*
source /opt/ros/humble/setup.bash 
```
</details>

<details open>
<summary><b>Preparation of the environment:</b></summary> 
To avoid problems with other projects or ROS environments that use the same physical network, it is highly recommended to separate the working computer from the rest by assigning a different ROS domain id. By not doing it, all the devices connected to the network may be able to use any topic, service or action launched from a different computer.


-	To set up a ROS domain ID to a group of commands:
```
export ROS_DOMAIN_ID=<your_domain_id>
```
-	Now you can run your nodes directly to that domain. To use ros2 commands from terminal in a different ID than the configured one:
```
ROS_DOMAIN_ID=<other_ID> ros2 <your_command> 
```
-	To maintain this setting between shell sessions:
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```
</details>

<details open>
<summary><b>The launch files:</b></summary> 

- **sim_control.launch.py:** Runs all related node, file or program related to the simulation, including simulation.py.

- **rover_control.launch.py:** Runs the files related to the Leo rover; decawave.py, leo_control.py, system_control_leo.py and ekf_node().

- **tello_control.launch.py:** Runs the files related to the Tello drone; decawave.py and system_control_tello.py.

- **complementary.launch.py:** Runs the non-essential files; demo.py, the program rviz2 and the rosbag functionality.

</details>

<details open>
<summary><b>Arguments:</b></summary> 

| Name of the argument | Description | Launch file available |
| :---: | :--- | :---: |
| first_start | This argument controls if the commands “\r\r” and “lep” are sent to the decawave interface when executing the decawave.py file. (**Default:** false) | total_control.launch.py |
| decawave_leo | This argument controls if the decawave node should be executed with the nodes related to the Leo rover. If using the total_control.launch.py, this node should be set to false. (**Default:** false) | rover_control.launch.py (can be inherited from total_control.launch.py) |
| decawave_tello | Similar to the decawave_leo argument but with the nodes related to the Tello drone. (**Default:** false) | tello_control.launch.py (can be inherited from total_control.launch.py) |
| complementary | This argument allows to use the set of non-essential nodes: rviz, demo.py and rosbag. (**Default:** true) | total_control.launch.py |
| rviz | This argument decides if rviz2 is executed. The complementary argument must be set to true. (**Default:** true) | complementary.launch.py (can be inherited from total_control.launch.py) |
| path_visual | This argument decides if demo.py is executed. The complementary argument must be set to true. (**Default:** true)| complementary.launch.py (can be inherited from total_control.launch.py) |
| rosbag | This argument decides if the rosbag functionality is executed. The complementary argument must be set to true. (**Default:** false) | complementary.launch.py (can be inherited from total_control.launch.py) |
| leo | This argument enables the execution of the nodes related to the Leo rover. Launches rover_control.launch.py (**Default:** true) | total_control.launch.py |
| tello | This argument enables the execution of the nodes related to the Tello drone. Launches tello_control.launch.py (**Default:** false) | total_control.launch.py |

</details>

<details open>
<summary><b>Running the simulation:</b></summary> 

The simulation can be executed using:
```
ros2 launch real_hardware sim.launch.py
```

To take off and land the drone:
```
ros2 service call /drone1/tello_action tello_msgs/srv/TelloAction "{cmd: 'takeoff'}" # or cmd: 'land'
```

And to simulate the hot swap subroutine:
```
ros2 topic pub -1 /hotswap std_msgs/msg/Empty "{}"
```

<details open>
<summary><b>Running the real hardware:</b></summary> 
One the other hand, before launching any file in the real hardware, we need to make sure that our system is ready to execute it properly:

1. The Leo rover is turned on and placed at the start point.
2. Using MobaXTerm, we have ssh connection to the Jetson Orin Nano.
3. In a second tab of the MobaXTerm, we have connection to the raspberry pi equipped in the Leo rover.
4. The Master PC and the Jetson Orin Nano dev kit are connected using ethernet connection, with the same ROS_DOMAIN_ID (by default falls into the 0).
5. Our decawave interface is connected using USB to the Master PC.
6. The drone is turned on, placed at the starting position and connected using WIFI to the Jetson Orin Nano dev kit.
7. The drone carries one decawave antenna (modified) that is turned on.
8. The rover carries one decawave antenna that is turned on.
9. Every board is in the workspace created and with the packages correctly sourced.

Now that the boards are set up, ROS is prepared in the different machines and the workspaces have the packages ready, we can execute the correct launch files with the correct arguments.

- On the Jetson Orin Nano:
```
roslaunch leorover frames_launch.launch
```
- On the raspberry pi:
```
roslaunch conversion_pkg connection.launch
```
- On the Master PC:
```
ros2 launch real_hardware total_control.launch.py
```
- On the Jetson Orin Nano dev kit:
```
ros2 launch real_hardware tello_control.launch.py decawave_tello:=false
```
</details>
