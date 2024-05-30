import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from tf2_ros import Buffer, TransformListener
import threading
import math

class VelControlledRobot(Node):

    def __init__(self):
        super().__init__('velocity_controlled_robot')

        # Get the publish topic
        self.target_pub = self.declare_parameter(
          'target_pub', '/drone1/cmd_vel').get_parameter_value().string_value
        
        # Get the frame of the drone
        self.target_drone = self.declare_parameter(
          'target_drone', 'base_link_1').get_parameter_value().string_value
        
        # Get the topic where the goals are received
        self.target_topic = self.declare_parameter(
          'target_topic', '/drone1/new_goal').get_parameter_value().string_value
        
        # Create subscriber for the goal
        self.subscription = self.create_subscription(Vector3, self.target_topic, self.goal_callback, 10)

        # Create publisher for the output velocity
        self.publisher_ = self.create_publisher(Twist, self.target_pub, 1000)

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables
        self.target_x = 100.0
        self.target_y = 100.0
        self.flag_move = False
        self.flag_home = True

    def publish_thread(self):
        timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # Get the position of the drone
        try:
            self.transform = self.tf_buffer.lookup_transform('map', self.target_drone, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to get TF transform: {str(e)}")
            return

        # Create the target position
        if self.flag_home or True: # ignore unsended positions
            target_position = Point(x=0.5, y=0.5, z=1.0)
            
            # If the drone should move...
            if self.flag_move:
                target_position.x = self.target_x
                target_position.y = self.target_y
                target_position.z = 1.0
                self.get_logger().info('Moving to target: x=%f, y=%f, z=%f' % (target_position.x, target_position.y, target_position.z))
            # If the drone should go home...
            """else:
                last = self.target_drone[-1]
                match last:
                    case '1':
                        HomePosition = -2.0
                    case '2':
                        HomePosition = -1.0
                    case '3':
                        HomePosition = 1.0
                    case '4':
                        HomePosition = 2.0
                    case _:
                        HomePosition = 0.0
                        self.get_logger().info('Sending unknown drone to Home (0, 0)')

                target_position.x = HomePosition
                target_position.y = 0.0
                target_position.z = self.transform.transform.translation.z"""

            # Calculate the velocities
            cmd_vel = self.calculate_velocity(self.transform, target_position)
            # self.get_logger().info('At a speed of: x=%f, y=%f, z=%f' % (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z))

        else:
            cmd_vel = Twist()
            # self.get_logger().warning('No target receibed')

        # Publish the velocity command to move the robot
        self.publisher_.publish(cmd_vel)

    def goal_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y

        # Avoid target too far or too close to origin
        """if(abs(self.target_x) >= 100) or (abs(self.target_y) >= 100):
            self.flag_move = False
            self.flag_home = True
        else:"""
        self.flag_move = True

    def calculate_velocity(self, transform, target_position):
        # Get actual position of the robot
        robot_position = transform.transform.translation

        # Calculate direction vector and distance to the objective
        direction = Point()
        direction.x = target_position.x - robot_position.x
        direction.y = target_position.y - robot_position.y
        direction.z = target_position.z - robot_position.z

        distance = math.sqrt(direction.x**2 + direction.y**2 + direction.z**2)

        # Compute linear velocities
        k_linear = 0.1
        linear_speed = k_linear * distance

        # Create the msg to move the robot. If the goal is reached, stop the robot and the node
        twist_msg = Twist()
        if distance < 0.2:
            # self.get_logger().info('Goal  x=%f, y=%f, z=%f reached!' % (target_position.x, target_position.y, target_position.z))
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
        else:
            twist_msg.linear.x = min(linear_speed * direction.x / distance, 0.5)
            twist_msg.linear.y = min(linear_speed * direction.y / distance, 0.5)
            twist_msg.linear.z = min(linear_speed * direction.z / distance, 0.5)

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
