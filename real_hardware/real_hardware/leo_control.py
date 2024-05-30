

from __future__ import print_function
import roslibpy
import roslibpy.actionlib
import roslibpy.tf
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Point
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster
import math
from nav_msgs.msg import Odometry
from custom_interfaces.msg import DecawaveList
from transformations import euler_from_quaternion
import numpy as np 
from tf2_ros import Buffer, TransformListener
from real_hardware.PID import PIDController as pid
import time

# Create a connection between the Leo rover and the computer
print('Connecting to Leo Rover on "10.0.0.1:8765"')
client = roslibpy.Ros(host='10.0.0.1', port=8765)
client.run()    
print('Connected to Leo Rover')

tf_msg = TransformStamped()
        
class odom_builder(Node):
    def __init__(self):
        super().__init__('odom_builder')
        # This node takes the data from the leorover and publishes it so the EKF node can use it
        
        # Publisher for the odometry data coming from the camera
        self.publisher_odom_robot = self.create_publisher(Odometry, '/odom/robot', 1000)
        
        # Publisher for the odometry data coming from the beacons
        self.publisher_odom_gps = self.create_publisher(Odometry, '/odom/gps', 1000)
        
        # Subscriber to the beacons data (already published in ROS2 in the computer)
        self.positions_deca = self.create_subscription(DecawaveList, '/decawave_data', self.deca_callback, 10)
              
        # Subscriber to the odometry of the camera (published in ROS1 at the leo)
        self.odom_robot = roslibpy.Topic(client, '/camera/odom/sample', 'nav_msgs/Odometry')
        self.odom_robot.subscribe(self.odom_robot_callback)
        
        # The EKF node also needs the full TF tree (published in ROS1 at the leo)
        self.tf_listener = roslibpy.Topic(client, '/tf', 'tf2_msgs/TFMessage')
        self.tf_listener.subscribe(self.full_tf_callback)
        
        # TF broadcaster in ROS2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # TF listener in ROS1
        self.tf_odom_baselink_sub = roslibpy.Topic(client, '/str_tf_listener', 'std_msgs/String')
        self.tf_odom_baselink_sub.subscribe(self.tf_odom_baselink)
        
    def odom_robot_callback(self, msg):
        # Take every message of the odometry given by the camera, create a ROS2 nav_msg Odometry and publish it
        odom_robot = Odometry()
        odom_robot.header.stamp = self.get_clock().now().to_msg()
        odom_robot.header.frame_id = 'odom'
        odom_robot.child_frame_id = 'base_footprint'
        odom_robot.pose.pose.position.x = msg['pose']['pose']['position']['x']
        odom_robot.pose.pose.position.y = msg['pose']['pose']['position']['y']
        odom_robot.pose.pose.position.z = msg['pose']['pose']['position']['z']
        odom_robot.pose.pose.orientation.x = msg['pose']['pose']['orientation']['x']
        odom_robot.pose.pose.orientation.y = msg['pose']['pose']['orientation']['y']
        odom_robot.pose.pose.orientation.z = msg['pose']['pose']['orientation']['z']
        odom_robot.pose.pose.orientation.w = msg['pose']['pose']['orientation']['w']
        odom_robot.pose.covariance = msg['pose']['covariance']
        odom_robot.twist.twist.linear.x = msg['twist']['twist']['linear']['x']
        odom_robot.twist.twist.linear.y = msg['twist']['twist']['linear']['y']
        odom_robot.twist.twist.linear.z = msg['twist']['twist']['linear']['z']
        odom_robot.twist.twist.angular.x = msg['twist']['twist']['angular']['x']
        odom_robot.twist.twist.angular.y = msg['twist']['twist']['angular']['y']
        odom_robot.twist.twist.angular.z = msg['twist']['twist']['angular']['z']
        odom_robot.twist.covariance = msg['twist']['covariance']
        self.publisher_odom_robot.publish(odom_robot)
        
    def deca_callback(self, msg):
        # Take every message of the position given by the beacons, create a ROS2 nav_msg Odometry and publish it
        odom_gps = Odometry()
        odom_gps.header.stamp = self.get_clock().now().to_msg()
        odom_gps.header.frame_id = 'room_map'
        odom_gps.child_frame_id = 'base_footprint'        
        
        # From all the tags, we only need the one of the rover
        for i in range(msg.amount):
            if msg.decawaveposition[i].name == "488E":  # 488E
                odom_gps.pose.pose.position.x = msg.decawaveposition[i].x
                odom_gps.pose.pose.position.y = msg.decawaveposition[i].y
                odom_gps.pose.pose.position.z = msg.decawaveposition[i].z   

        self.publisher_odom_gps.publish(odom_gps)

    def full_tf_callback(self, msg):
        # Listen to every non-static TF in the leo (ROS1) and broadcast it in ROS2
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg['transforms'][0]['header']['frame_id']
        t.child_frame_id = msg['transforms'][0]['child_frame_id']
        t.transform.translation.x = msg['transforms'][0]['transform']['translation']['x']
        t.transform.translation.y = msg['transforms'][0]['transform']['translation']['y']
        t.transform.translation.z = msg['transforms'][0]['transform']['translation']['z']
        t.transform.rotation.x = msg['transforms'][0]['transform']['rotation']['x']
        t.transform.rotation.y = msg['transforms'][0]['transform']['rotation']['y']
        t.transform.rotation.z = msg['transforms'][0]['transform']['rotation']['z']
        t.transform.rotation.w = msg['transforms'][0]['transform']['rotation']['w']
        self.tf_broadcaster.sendTransform(t)
             
    def tf_odom_baselink(self, msg):
        # Listen to the TF between the odom frame and the rover base_link frame (internal variable)
        global tf_msg
        data = msg["data"]
        tf = data.split(",")
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = float(tf[0])
        tf_msg.transform.translation.y = float(tf[1])
        tf_msg.transform.translation.z = float(tf[2])
        tf_msg.transform.rotation.x = float(tf[3])
        tf_msg.transform.rotation.y = float(tf[4])
        tf_msg.transform.rotation.z = float(tf[5])
        tf_msg.transform.rotation.w = float(tf[6])       

class leo_controller(Node):
    def __init__(self):
        super().__init__('leo_controller')
        # Create publisher
        self.talker = roslibpy.Topic(client, '/str_cmd_vel', 'std_msgs/String')

        # Create subscriber to goal
        self.subscription = self.create_subscription(Vector3, '/new_goal/leo', self.goal_callback, 10)
        
        # Create subscriber for the filtered position of the rover given by the EKF node
        self.positions_deca = self.create_subscription(Odometry, '/odom/filtered', self.deca_callback, 10)

        # Variables
        self.target = Point()
        self.pos_leo = Vector3()
        self.dt = 0.2
        self.x_controller = pid(kp=0.5, ki=0.0, kd=0.0)
        self.alpha_controller = pid(angle=True, kp=1.0, ki=0.0, kd=0.0)
        self.msg_time = time.time()
        
    def deca_callback(self, msg: Odometry):
        # Save the position of the leo
        self.pos_leo.x = msg.pose.pose.position.x
        self.pos_leo.y = msg.pose.pose.position.y
        self.pos_leo.z = msg.pose.pose.position.z

    # Store the next goal point in a global variable
    def goal_callback(self, msg):
        self.target.x = msg.x
        self.target.y = msg.y
        # self.get_logger().info('New goal receibed: x = %f, y = %f' % (msg.x, msg.y))
    
        if self.target.x != 0 and self.target.y != 0:
            # Calculate the velocities
            cmd_vel = self.calculate_velocity(self.target, self.pos_leo)
            # self.get_logger().info('At a speed of: x=%f, yaw=%f' % (cmd_vel.linear.x, self.cmd_vel.angular.z))
        else:
            cmd_vel = Twist()
            # self.get_logger().warning('No target receibed')

        # Publish the velocity command to move the robot
        #self.get_logger().info('Velocity is: vx = %f, wz = %f' % (cmd_vel.linear.x, cmd_vel.angular.z))
        cmd = str(cmd_vel.linear.x) + ', ' + str(cmd_vel.angular.z)
        self.talker.publish(roslibpy.Message({'data': cmd}))
            
    def calculate_velocity(self, target_position, robot_position):
        x_speed = 0.0
        alpha_speed = 0.0
        
        # Calculate direction vector and distance to the objective
        direction = Point()
        direction.x = target_position.x - robot_position.x
        direction.y = target_position.y - robot_position.y
        distance = math.sqrt(direction.x**2 + direction.y**2)
        
        # get current and target yaw
        euler_angles = euler_from_quaternion([tf_msg.transform.rotation.w,
                                             tf_msg.transform.rotation.x,
                                             tf_msg.transform.rotation.y,
                                             tf_msg.transform.rotation.z])
        current_yaw = euler_angles[2]        
        target_yaw = math.atan2(direction.y, direction.x)
        
        # Limit the target yaw between [-pi, pi]
        while(target_yaw < -math.pi):
            target_yaw += 2 * math.pi
        while(target_yaw > math.pi):
            target_yaw -= 2 * math.pi
        
        #self.get_logger().info('Current_yaw: ' + str(current_yaw))
        #self.get_logger().info('Target_yaw:  ' + str(target_yaw))

        # Compute time between messages and compute the PID control action acordingly
        if self.dt == 0:
            self.dt = time.time() - self.msg_time
            self.msg_time = time.time()
        else:
            self.dt = time.time() - self.msg_time
            self.msg_time = time.time()
            x_speed = self.x_controller.calc(0.0, distance, self.dt, 0)
            alpha_speed = self.alpha_controller.calc(current_yaw, target_yaw, self.dt, 0)     
        
        # Create the msg to move the robot. If the goal is reached, stop the robot
        twist_msg = Twist()
        if distance < 0.1:
            # self.get_logger().info('Goal  x=%f, y=%f reached!' % (target_position.x, target_position.y))
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            twist_msg.linear.x = np.clip(x_speed, -1, 1)
            twist_msg.angular.z = np.clip(alpha_speed, -1, 1)
            
        # If the error on the orientation is too large, do not move forward, just rotate
        if abs(target_yaw - current_yaw) >= 0.15:
            twist_msg.linear.x = 0.0
            
        return twist_msg
           
def main(args=None):
    rclpy.init(args=args)
    
    try:
        odom_builder_node = odom_builder()
        leo_controller_node = leo_controller()

        executor = MultiThreadedExecutor()
        executor.add_node(odom_builder_node)
        executor.add_node(leo_controller_node)
        
        try: 
            executor.spin()
        finally:
            executor.shutdown()
            odom_builder_node.destroy_node()
            leo_controller_node.destroy_node()
            
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
