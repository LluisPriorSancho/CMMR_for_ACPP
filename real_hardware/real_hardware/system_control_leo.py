import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.msg import DecawaveList
from nav_msgs.msg import Odometry
import pandas as pd
from std_msgs.msg import Bool
import math


# Get list of positions to visit
order_leo = pd.read_csv('~/ros2_ws/src/TFM/solo_files/wavefront/map_order_rover.csv').to_numpy()

# Global variables
pos_leo = Vector3()
is_hotswapping = False

class position_supervisor_leo(Node):
    def __init__(self):
        super().__init__('position_supervisor_leo')
        # Create subscriber for the position of the leo
        self.positions_deca = self.create_subscription(Odometry, '/odom/filtered', self.deca_callback, 10)

        # Create subscriber to know when a hotswap is happening
        self.hotswap = self.create_subscription(Bool, '/hotswapping', self.hotswap_callback, 10)
        
    # Store the position of the leo acording to the EKF node
    def deca_callback(self, msg: Odometry):
        global pos_leo
        pos_leo.x = msg.pose.pose.position.x
        pos_leo.y = msg.pose.pose.position.y
        pos_leo.z = msg.pose.pose.position.z
                
    # Variable to indicate that the hotswapping is happening
    def hotswap_callback(self, msg):
        global is_hotswapping
        is_hotswapping = msg.data

class goal_sender_leo(Node):
    def __init__(self):
        super().__init__('goal_sender_leo')
        
        # Variables
        global pos_leo
        global order_leo
        self.dt = 1.0
        self.hotswapping = False
        self.cell_number_leo = 0
        self.resolution = 0.7
        self.prev_goal = Vector3()
        self.goal_leo = Vector3()
        self.offset = Vector3(x=0.9-0.4, y=0.8-0.35)
        
        # Create timer
        timer = self.create_timer(self.dt, self.on_timer)

        # Create publishers for the goals of the leo
        self.publisher_leo = self.create_publisher(Vector3, '/new_goal/leo', 1000)

    def on_timer(self):
        global is_hotswapping
        
        # If the hotswap is happening, the new goal is the same position the leo is not (not moving)
        if is_hotswapping:
            self.goal_leo = pos_leo
        
        # If the hotswap is not happening
        else:
            # Get next goal
            self.goal_leo = self.get_next_goals()
            # self.get_logger().info('Leo: Getting new goal')
    
        # Publish the goal for the leo
        self.publisher_leo.publish(self.goal_leo)
        # self.get_logger().info('Leo: Publishing goal: x=' + str(self.goal_leo.x) + ' y=' + str(self.goal_leo.y))
            
    def get_next_goals(self):
        # Get distance to the objective
        distance_leo = self.get_distance(self.goal_leo, pos_leo, Vector3())
        
        # If it is close to the current goal, advance to the next cell index
        self.cell_number_leo = self.next_cell(distance_leo, self.cell_number_leo, order_leo)

        # Get exact position of the goal: cell*resolution + offset 
        # (the offset is to place the robot in the middle of the cell)
        next_goal_leo = self.next_goal(self.cell_number_leo, self.resolution, order_leo)
               
        return next_goal_leo
        
    # Function to get the distance between the actual position of the rover and the goal
    def get_distance(self, To: Vector3, From: Vector3, bias_to: Vector3):
        # Calculate direction vector and distance to the objective
        direction = Point()
        direction.x = To.x + bias_to.x - From.x 
        direction.y = To.y + bias_to.y - From.y 
        distance = math.sqrt(direction.x**2 + direction.y**2)
        return distance
    
    # Function to get the next cell-like goal is the distance is close
    def next_cell(self, distance: float, cell_number: int, order):
        if distance <= 0.3:
            cell_number += 1
            if cell_number >= order.shape[0]:  
                cell_number -= 1
                # print('Reached the goal after exploration')
        return cell_number
    
    # Function to get the goal in global coordinates
    def next_goal(self, cell_number: int, resolution: float, order):
        goal = Vector3()
        goal.x = ((order[cell_number, 0] - 1)+ 0.5) * resolution + self.offset.x 
        goal.y = ((order[cell_number, 1] - 1)+ 0.5) * resolution + self.offset.y 
        return goal
    
def main(args=None):
    rclpy.init(args=args)

    try:
        position_supervisor_leo_node = position_supervisor_leo()
        goal_sender_leo_node = goal_sender_leo()

        executor = MultiThreadedExecutor()
        executor.add_node(position_supervisor_leo_node)
        executor.add_node(goal_sender_leo_node)
        try: 
            executor.spin()
        finally:
            executor.shutdown()
            position_supervisor_leo_node.destroy_node()
            goal_sender_leo_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()