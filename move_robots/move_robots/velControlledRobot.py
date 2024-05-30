import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from tf2_ros import Buffer, TransformListener
import threading
import math 

class VelControlledRobot(Node):

    def __init__(self):
        super().__init__('velocity_controlled_robot')
        # Create subscriber for the goal
        self.subscription = self.create_subscription(Vector3, '/new_goal', self.goal_callback, 10)

        # Create publisher for the velocities
        self.publisher_ = self.create_publisher(Twist, '/leo1_velocity_controller/cmd_vel_unstamped', 1000)

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables
        self.target_x = 100.0
        self.target_y = 100.0
        self.M_PI = math.pi

    def publish_thread(self):
        timer = self.create_timer(0.2, self.on_timer)

    def on_timer(self):
        # Get rover position
        try:
            self.transform = self.tf_buffer.lookup_transform('map', 'rover1/base_footprint', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to get TF transform: {str(e)}")
            return

        # Create the target position
        if self.target_x != 100: # ignore unsended positions
            target_position = Point()
            target_position.x = self.target_x
            target_position.y = self.target_y
            target_position.z = self.transform.transform.translation.z
            # self.get_logger().info('Moving to target: x=%f, y=%f' % (self.target_x, self.target_y))

            # Calculate the velocities
            cmd_vel = self.calculate_velocity(self.transform, target_position)
            # self.get_logger().info('At a speed of: x=%f, yaw=%f' % (cmd_vel.linear.x, cmd_vel.angular.z))
        
        else:
            cmd_vel = Twist()
            # self.get_logger().warning('No target receibed')

        # Publish the velocity command to move the robot
        self.publisher_.publish(cmd_vel)

    def goal_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.get_logger().info('New goal receibed: x = %f, y = %f' % (msg.x, msg.y))

    def calculate_velocity(self, transform, target_position):
        # Get actual position of the robot
        robot_position = transform.transform.translation

        # Calculate direction vector and distance to the objective
        direction = Point()
        direction.x = target_position.x - robot_position.x
        direction.y = target_position.y - robot_position.y
        distance = math.sqrt(direction.x**2 + direction.y**2)

        # Compute angle between robot orientation and target orientation
        current_yaw = math.atan2(
            2 * (transform.transform.rotation.w *
                 transform.transform.rotation.z),
            1 - 2 * (transform.transform.rotation.z**2)
        )
        target_yaw = math.atan2(direction.y, direction.x)
        error_yaw = target_yaw - current_yaw
        
        # limit the range of the error to [-pi, pi]
        while(error_yaw < -self.M_PI):
                error_yaw += 2 * self.M_PI
        while(error_yaw > self.M_PI):
            error_yaw -= 2 * self.M_PI
            
        # Compute linear and angular velocity
        k_angle = 2.0
        k_distance = 2.0
        linear_speed = k_distance * distance
        angular_speed = k_angle * error_yaw

        # Create the msg to move the robot. If the goal is reached, stop the robot and the node
        twist_msg = Twist()
        if distance < 0.2:
            # self.get_logger().info('Goal  x=%f, y=%f reached!' % (target_position.x, target_position.y))
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.target_x = 100.0
            self.target_y = 100.0
        else:
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed

        return twist_msg

def main(args=None):
    rclpy.init(args=args)

    vel_controlled_robot = VelControlledRobot()

    publish_thread = threading.Thread(target=vel_controlled_robot.publish_thread)

    publish_thread.start()
    rclpy.spin(vel_controlled_robot) 

    publish_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
