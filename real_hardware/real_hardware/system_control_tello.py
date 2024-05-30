import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.msg import DecawaveList
import pandas as pd
from std_msgs.msg import Empty
from djitellopy import Tello
import math
import time
from real_hardware.PID import PIDController as pid
from std_msgs.msg import Bool
import numpy as np

# Get list of positions to visit
order_tello = pd.read_csv('~/ros2_ws/src/TFM/solo_files/wavefront/map_order_tello.csv').to_numpy()

# Global variables
pos_tello = Vector3()
pos_leo = Vector3()
tello_connected = False

# Function to connect to the drone
def connect():
    global tello_connected
    
    print('Tello: Connecting to drone')
    tello.connect()
    print('Tello: Connected to drone')
    tello_connected = True
    
    
# Configure drone connection
tello = Tello()
Tello.TELLO_IP='192.168.10.2'
Tello.RESPONSE_TIMEOUT = int(10.0)
connect()

class position_supervisor_tello(Node):
    def __init__(self):
        super().__init__('position_supervisor_tello')
        # Create subscriber to the position of the beacon for the drone
        self.positions_deca = self.create_subscription(DecawaveList, '/decawave_data', self.deca_callback, 10)

        # Create subscriber to connect to the drone
        self.connection_drone = self.create_subscription(Empty, '/connection', self.connection_callback, 10)         
        
        # Create subscriber to the position of the rover given by the EKF node
        self.positions_leo = self.create_subscription(Odometry, '/odom/filtered', self.ekf_callback, 10)
        
    # Save the position of the leo rover acording to the EKF node
    def ekf_callback(self, msg: Odometry):
        global pos_leo
        pos_leo.x = msg.pose.pose.position.x
        pos_leo.y = msg.pose.pose.position.y
        pos_leo.z = msg.pose.pose.position.z
        
    # Store the position of the tello acording to the decawave
    def deca_callback(self, msg):
        global pos_tello
        for i in range(msg.amount):
            if msg.decawaveposition[i].name != "488E":  # 488E
                pos_tello.x = msg.decawaveposition[i].x
                pos_tello.y = msg.decawaveposition[i].y
                pos_tello.z = msg.decawaveposition[i].z
    
    # When the hotswap is already done, a message is published to connect to the drone
    def connection_callback(self, msg):
        connect()

class tello_velcontrol(Node):
    def __init__(self):
        super().__init__('tello_velcontrol')
        # Create a subscriber to the tello goals
        self.subscription = self.create_subscription(Vector3, '/new_goal/tello', self.goal_callback, 10)

        # Variables
        global pos_tello
        self.target = Point()
        self.dt = 0
        self.x_controller = pid()
        self.y_controller = pid()
        self.msg_time = time.time()

    # Store the next goal point in a global variable
    def goal_callback(self, msg):   
        self.target.x = msg.x 
        self.target.y = msg.y

        # self.get_logger().info('New goal receibed: x = %f, y = %f' % (msg.x, msg.y))
        if self.target.x != 0 and self.target.y != 0:
            # While the drone is not close to the objective compute the commands to move it to the correct position
            velocity = self.calculate_velocity(self.target, pos_tello)
            if velocity.linear.x != 0 or velocity.linear.y != 0 or velocity.linear.z != 0:
                self.send_command(velocity)

    def calculate_velocity(self, target_position, robot_position):
        x_speed = 0.0
        y_speed = 0.0
            
        # Calculate direction vector and distance to the objective
        direction = Point()
        direction.x = target_position.x - robot_position.x
        direction.y = target_position.y - robot_position.y
        distance = math.sqrt(direction.x**2 + direction.y**2)
        
        # Compute time between messages to apply a PID to their x and y velocities
        if self.dt == 0:
            self.dt = time.time() - self.msg_time
            self.msg_time = time.time()
        else:
            self.dt = time.time() - self.msg_time
            self.msg_time = time.time()
            x_speed = self.x_controller.calc(robot_position.x, target_position.x, self.dt, 0)
            y_speed = self.y_controller.calc(robot_position.y, target_position.y, self.dt, 0)     
            
        current_altitude = tello.get_height()
        # self.get_logger().info('Tello: Altitude is: ' + str(current_altitude))
        z_speed = float(130.0 - current_altitude)

        # Create the msg to move the robot. If the goal is reached, stop the robot and the node
        twist_msg = Twist()
        if distance < 0.3:
            # self.get_logger().info('Goal  x=%f, y=%f reached!' % (target_position.x, target_position.y))
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
        else:
            twist_msg.linear.x = np.clip(x_speed, -15, 15)
            twist_msg.linear.y = np.clip(y_speed, -15, 20)
            twist_msg.linear.z = np.clip(z_speed, -15, 15)
        return twist_msg
    
    # Send the velocities to the drone using the official API
    def send_command(self, cmd_vel):
        if tello_connected:
            tello.send_rc_control(int(-cmd_vel.linear.y), 
                                int(cmd_vel.linear.x), 
                                int(cmd_vel.linear.z), 
                                int(-cmd_vel.angular.z))

class goal_sender_tello(Node):
    def __init__(self):
        super().__init__('goal_sender_tello')

        # Variables
        global pos_tello
        global pos_leo
        global order_tello
        self.goal_tello = Vector3()
        self.dt = 1.0
        self.hotswapping = False
        self.cell_number_tello = 0
        self.resolution = 0.7
        self.battery_limit = 25

        # Create timer
        timer = self.create_timer(self.dt, self.on_timer)

        # Create publishers of positions
        self.publisher_tello = self.create_publisher(Vector3, '/new_goal/tello', 1000)
        self.publisher_hotswap = self.create_publisher(Bool, '/hotswapping', 1000)

    def on_timer(self):
        global tello_connected
        
        # When the tello is connected:
        if tello_connected:
            # When the drone is flying
            if tello.is_flying:
                # Correct yaw if necessary
                self.correct_yaw()
                
                # Print battery
                bat = tello.get_battery()
                self.get_logger().info('Tello: Baterry is: ' + str(bat))

                # If battery is low, the next goal of the tello is the position of the leo
                if bat <= self.battery_limit:
                    # If the position of the leo is not available, go home 
                    if pos_leo.x == 0 and pos_leo.y == 0:
                        self.goal_tello.x = 0.9
                        self.goal_tello.y = 0.8
                        self.get_logger().info('Tello: Leo not available, returning home')
                    else:
                        self.goal_tello = pos_leo
                    self.hotswapping = True
                    self.get_logger().info('Tello: Hotswapping')
                    
                # If the battery is good enough, continue to the next goal
                else:
                    # Get next goals for both robots
                    self.goal_tello = self.get_next_goals()

                # When battery is low and drone is on top of leo, land
                if self.hotswapping:
                    distance = self.get_distance(self.goal_tello, pos_tello)
                    self.publisher_hotswap.publish(msg=Bool(data=True))
                    if distance <= 0.4:
                        tello.land()
                        self.get_logger().info('Tello: Landing the drone for hotswapping')
                        tello_connected = False
                        
            # If the drone is not flying
            else:
                if tello_connected:
                    # Take off drone and move up 1 meter
                    tello.takeoff()  
                    tello.move_up(60)
                    self.get_logger().info('Tello: Taking off the drone')
                    self.correct_yaw()
                    self.hotswapping = False
                    self.publisher_hotswap.publish(msg=Bool(data=False))

            # Publish new goals
            self.publisher_tello.publish(self.goal_tello)
        
    # Function to get the current yaw of the drone and to corrected if necessary
    def correct_yaw(self):
        actual_yaw = tello.get_yaw()
        if actual_yaw >= 5:
            tello.send_rc_control(0, 0, 0, 0)
            tello.rotate_counter_clockwise(actual_yaw)
            self.get_logger().info('Tello: Correcting drone heading')

    def get_next_goals(self):
        # Get distance to the objective
        distance_tello = self.get_distance(self.goal_tello, pos_tello)

        # If it is close to the current goal, advance to the next cell index
        self.cell_number_tello = self.next_cell(distance_tello, self.cell_number_tello, order_tello)

        # Get exact position of the goal: cell*resolution + offset 
        # (the offset is to place the robot in the middle of the cell)
        next_goal_tello = self.next_goal(self.cell_number_tello, self.resolution, order_tello)
               
        return next_goal_tello
        
    # Function to get the distance between the actual position of the drone and the goal
    def get_distance(self, To, From):
        # Calculate direction vector and distance to the objective
        direction = Point()
        direction.x = To.x - From.x
        direction.y = To.y - From.y
        distance = math.sqrt(direction.x**2 + direction.y**2)
        return distance
    
    # Function to get the next cell-like goal is the distance is close
    def next_cell(self, distance: float, cell_number: int, order):
        if distance <= 0.4:
            cell_number += 1
            if cell_number >= order.shape[0]:  
                cell_number -= 1
                print('Reached the goal after exploration')
                tello.land()
        return cell_number
        
    # Function to get the goal in global coordinates
    def next_goal(self, cell_number: int, resolution: float, order):
        goal = Vector3()
        goal.x = (order[cell_number, 0] + 0.5) * resolution + 0.25
        goal.y = (order[cell_number, 1] + 0.5) * resolution + 0.15
        return goal
    
def main(args=None):
    rclpy.init(args=args)

    try:
        tello_velcontrol_node = tello_velcontrol()
        position_supervisor_tello_node = position_supervisor_tello()
        goal_sender_tello_node = goal_sender_tello()

        executor = MultiThreadedExecutor()
        executor.add_node(tello_velcontrol_node)
        executor.add_node(position_supervisor_tello_node)
        executor.add_node(goal_sender_tello_node)
        try: 
            executor.spin()
        finally:
            executor.shutdown()
            tello_velcontrol_node.destroy_node()
            position_supervisor_tello_node.destroy_node()
            goal_sender_tello_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
