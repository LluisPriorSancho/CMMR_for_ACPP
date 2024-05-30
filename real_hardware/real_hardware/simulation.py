import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import MultiThreadedExecutor
import pandas as pd
from std_msgs.msg import Empty
import math
import copy

# Get list of positions to visit
order_leo = pd.read_csv('~/ros2_ws/src/TFM/solo_files/wavefront/map_order_rover.csv').to_numpy()
order_tello = pd.read_csv('~/ros2_ws/src/TFM/solo_files/wavefront/map_order_tello.csv').to_numpy()

class simulation(Node):
    def __init__(self):
        super().__init__('robots_position')
        # Create subscriber for the positions
        self.tello_pos_sub = self.create_subscription(Point, '/tello1_position', self.tello_pos_callback, 10)
        self.leo_pos_sub = self.create_subscription(Point, '/leo1_position', self.leo_pos_callback, 10)
        
        # Create a subscriber that simulates the hotswap
        self.hot_swap = self.create_subscription(Empty, '/hotswap', self.hotswap_callback, 10)

        # Create publishers for the goals
        self.publisher_tello = self.create_publisher(Vector3, '/drone1/new_goal', 1000)
        self.publisher_leo = self.create_publisher(Vector3, '/new_goal', 1000)
        
        # Create timer
        timer = self.create_timer(0.2, self.on_timer)

        # Variables
        self.pos_tello = Vector3()
        self.pos_leo = Vector3()
        self.goal_tello = Vector3()
        self.goal_leo = Vector3()
        self.prev_goal_leo = Vector3()
        self.prev_goal_tello = Vector3()
        self.cell_number_leo = 0
        self.cell_number_tello = 0
        self.hotswap = False
        
    # To indicate if the hotswap should ocurr, triggers manually.
    def hotswap_callback(self, msg):
        if self.hotswap:
            self.hotswap = False
        else:
            self.hotswap = True
        
    # To get the position of the tello drone in gazebo
    def tello_pos_callback(self, msg: Point):
        self.pos_tello.x = msg.x
        self.pos_tello.y = msg.y
        self.pos_tello.z = msg.z
        
    # To get the position of the leo rover in gazebo
    def leo_pos_callback(self, msg: Point):
        self.pos_leo.x = msg.x
        self.pos_leo.y = msg.y
        self.pos_leo.z = msg.z
        
    def on_timer(self):
        # Get the distance between the goal positions and the actual position of the robots
        distance_leo = self.get_distance(self.goal_leo, self.pos_leo)
        distance_tello = self.get_distance(self.goal_tello, self.pos_tello)
            
        # If the hotswap is not happening:
        if not self.hotswap:    
            # get the next goal cell        
            self.cell_number_leo = self.next_cell(distance_leo, self.cell_number_leo, order_leo)
            self.cell_number_tello = self.next_cell(distance_tello, self.cell_number_tello, order_tello)
            
            # Transform from cell-like goal to global-like goal
            self.goal_leo = self.next_goal(self.cell_number_leo, 1.0, order_leo)
            self.goal_tello = self.next_goal(self.cell_number_tello, 1.0, order_tello)
        
        # If the hotswap is happening:
        else:
            # The goal of the leo is the same exact position it is at that moment
            self.goal_leo = self.goal_leo
            
            # The goal of the tello is the same position of the leo plus an offset
            self.goal_tello = self.pos_leo
            self.goal_tello.x = self.pos_leo.x + 0.4
        
        # Publish the goals only when they are diferent from the previous ones
        if self.different_goals(self.prev_goal_leo, self.goal_leo):
            self.publisher_leo.publish(self.goal_leo)
            self.prev_goal_leo = copy.deepcopy(self.goal_leo)

        if self.different_goals(self.prev_goal_tello, self.goal_tello):
            self.publisher_tello.publish(self.goal_tello)
            self.prev_goal_tello = copy.deepcopy(self.goal_tello)
            
    # Function to check if the goals are different
    def different_goals(self, previous: Point, new: Point):
        if (previous.x == new.x) and (previous.y == new.y):
            return False
        else: 
            return True
        
    # Function to compute the distance from the position of the robot ant the goal
    def get_distance(self, To: Vector3, From: Vector3, bias_to: Vector3 = Vector3()):
        # Calculate direction vector and distance to the objective
        direction = Point()
        direction.x = To.x + bias_to.x - From.x 
        direction.y = To.y + bias_to.y - From.y 
        distance = math.sqrt(direction.x**2 + direction.y**2)
        return distance
    
    # Function to get the next cell goal
    def next_cell(self, distance: float, cell_number: int, order):
        if distance <= 0.5:
            cell_number += 1
            if cell_number >= order.shape[0]:  
                cell_number -= 1
                # print('Reached the goal after exploration')
        return cell_number
    
    # Function to get the next goal in global coordinates
    def next_goal(self, cell_number: int, resolution: float, order):
        goal = Vector3()
        goal.x = ((order[cell_number, 0] - 1)+ 0.5) * resolution
        goal.y = ((order[cell_number, 1] - 1)+ 0.5) * resolution
        return goal
    
def main(args=None):
    rclpy.init(args=args)

    try:
        path_maker_node = simulation()
        executor = MultiThreadedExecutor()
        executor.add_node(path_maker_node)
        try: 
            executor.spin()
        finally:
            executor.shutdown()
            path_maker_node.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()