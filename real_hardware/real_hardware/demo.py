import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
    
class path_maker(Node):
    def __init__(self):
        super().__init__('path_maker')
        # Create subscribers for the positions (EKF, Beacons, Camera)
        self.tello_pos_sub = self.create_subscription(Odometry, '/odom/filtered', self.odom_filtered_callback, 10)
        self.tello_pos_sub = self.create_subscription(Odometry, '/odom/gps', self.odom_gps_callback, 10)
        self.tello_pos_sub = self.create_subscription(Odometry, '/odom/robot', self.odom_robot_callback, 10)
        
        # Create publishers of the markers for Rviz
        self.publisher_path_odom_filtered = self.create_publisher(Marker, '/marker/odometry/filtered', 10000)
        self.publisher_path_odom_gps = self.create_publisher(Marker, '/marker/odometry/gps', 10000)
        self.publisher_path_odom_robot = self.create_publisher(Marker, '/marker/odometry/robot', 10000)
        
        # Marker for the positions given by the EKF node (green)
        self.path_filtered = Marker()  
        self.path_filtered.type = 4
        self.path_filtered.color.g = 1.0
        self.path_filtered.color.a = 1.0
        self.path_filtered.scale.x = 0.05
        self.path_filtered.header.frame_id = 'room_map'
        self.path_filtered.header.stamp = self.get_clock().now().to_msg()
        self.cont_filtered = 0
        
        # Marker for the positions given by the beacons node (red)
        self.path_gps = Marker()  
        self.path_gps.type = 4
        self.path_gps.color.r = 1.0
        self.path_gps.color.a = 1.0
        self.path_gps.scale.x = 0.05
        self.path_gps.header.frame_id = 'room_map'
        self.path_gps.header.stamp = self.get_clock().now().to_msg()
        self.cont_gps = 0
        
        # Marker for the position given by the odometry of the camera (blue)
        self.path_robot = Marker()  
        self.path_robot.type = 4
        self.path_robot.color.b = 1.0
        self.path_robot.color.a = 1.0
        self.path_robot.scale.x = 0.05
        self.path_robot.header.frame_id = 'map'
        self.path_robot.header.stamp = self.get_clock().now().to_msg()
        self.cont_robot = 0
                
        
    def odom_filtered_callback(self, msg: Odometry):
        # Every ten messages, add a new point to the list.
        # This avoid creating large set of points
        if self.cont_filtered == 10:
            self.cont_filtered = 0
                        
            point = Point()
            point.x = msg.pose.pose.position.x
            point.y = msg.pose.pose.position.y
            point.z = msg.pose.pose.position.z
            self.path_filtered.points.append(point)
            self.publisher_path_odom_filtered.publish(self.path_filtered)
        else:
            self.cont_filtered += 1
        
    def odom_gps_callback(self, msg: Odometry):
        # Add every message to the list of points
        if self.cont_gps == 0:
            self.cont_gps = 0
            point = Point()
            point.x = msg.pose.pose.position.x
            point.y = msg.pose.pose.position.y
            point.z = msg.pose.pose.position.z
            self.path_gps.points.append(point)
            self.publisher_path_odom_gps.publish(self.path_gps)
        else:
            self.cont_gps += 1
        
    def odom_robot_callback(self, msg: Odometry):
        # Every ten messages, add a new point to the list.
        # This avoid creating large set of points
        if self.cont_robot == 10:
            self.cont_robot = 0
            point = Point()
            point.x = msg.pose.pose.position.x
            point.y = msg.pose.pose.position.y
            point.z = msg.pose.pose.position.z
            self.path_robot.points.append(point)
            self.publisher_path_odom_robot.publish(self.path_robot)
        else:
            self.cont_robot += 1
        
class marker_maker(Node):
    def __init__(self):
        super().__init__('marker_maker')      
        # Create subscriptions for the goal posiitons of the robots
        self.goal_leo = self.create_subscription(Vector3, '/new_goal/leo', self.goals_leo_callback, 10)
        self.goal_tello = self.create_subscription(Vector3, '/new_goal/tello', self.goals_tello_callback, 10)
        
        # Create publishers for the markers in Rviz
        self.publish_marker_leo = self.create_publisher(MarkerArray, '/marker/goal/leo', 10000)
        self.publish_marker_tello = self.create_publisher(MarkerArray, '/marker/goal/tello', 10000)
        
        # Goals of the rover (orange)
        self.prev_goal_leo = Vector3()
        self.marker_array_leo = MarkerArray()
        self.cont_leo = 0 
        
        # Goals of the drone (purple)
        self.prev_goal_tello = Vector3()
        self.marker_array_tello = MarkerArray()
        self.cont_tello = 0 
        
    def goals_leo_callback(self, msg: Vector3):
        # If the goal is different from the previous one, add it to the set
        if msg.x != self.prev_goal_leo.x or msg.y != self.prev_goal_leo.y:
            marker = Marker()
            marker.pose.position.x = msg.x
            marker.pose.position.y = msg.y
            marker.header.frame_id = 'room_map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = self.cont_leo
            marker.type = 2
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = 254.0/255.0
            marker.color.g = 241.0/255.0
            marker.color.b = 96.0/255.0
            marker.color.a = 1.0
            self.marker_array_leo.markers.append(marker)
            self.publish_marker_leo.publish(self.marker_array_leo)
            self.prev_goal_leo.x = msg.x
            self.prev_goal_leo.y = msg.y
            self.cont_leo += 1
            
    def goals_tello_callback(self, msg: Vector3):
        # If the goal is different from the previous one, add it to the set
        if msg.x != self.prev_goal_tello.x or msg.y != self.prev_goal_tello.y:
            marker = Marker()
            marker.pose.position.x = msg.x
            marker.pose.position.y = msg.y
            marker.header.frame_id = 'room_map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = self.cont_tello
            marker.type = 2
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = 191.0/255.0
            marker.color.g = 85.0/255.0
            marker.color.b = 236.0/255.0
            marker.color.a = 1.0
            self.marker_array_tello.markers.append(marker)
            self.publish_marker_tello.publish(self.marker_array_tello)
            self.prev_goal_tello.x = msg.x
            self.prev_goal_tello.y = msg.y
            self.cont_tello += 1
            
def main(args=None):
    rclpy.init(args=args)
    try:
        path_maker_node = path_maker()
        marker_maker_node = marker_maker()

        executor = MultiThreadedExecutor()
        executor.add_node(path_maker_node)
        executor.add_node(marker_maker_node)
        try: 
            executor.spin()
        finally:
            executor.shutdown()
            path_maker_node.destroy_node()
            marker_maker_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()