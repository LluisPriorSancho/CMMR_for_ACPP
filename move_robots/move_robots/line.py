import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3


class StraightLineCalculator(Node):

    def __init__(self):
        super().__init__('straight_line_calculator')
        # Create subscription to the final goal point
        self.subscription = self.create_subscription(
            Point,
            '/full_goal',
            self.goal_callback,
            10
        )
        
        # Create publishers for each drone
        self.publisher_drone1 = self.create_publisher(Vector3, '/drone1/new_goal', 10)
        self.publisher_drone2 = self.create_publisher(Vector3, '/drone2/new_goal', 10)
        self.publisher_drone3 = self.create_publisher(Vector3, '/drone3/new_goal', 10)
        self.publisher_drone4 = self.create_publisher(Vector3, '/drone4/new_goal', 10)

    def goal_callback(self, msg):
        self.get_logger().info('Received goal point: x=%f, y=%f, z=%f' % (msg.x, msg.y, msg.z))

        # Compute points in a straight line between received goal and origin
        for i in range(1, 5):
            x = msg.x / 5 * i
            y = msg.y / 5 * i
            z = msg.z / 5 * i
            point = Vector3(x=x, y=y, z=z)

            # Publish computed point to the respective drone's topic
            if i == 1:
                self.publisher_drone1.publish(point)
            elif i == 2:
                self.publisher_drone2.publish(point)
            elif i == 3:
                self.publisher_drone3.publish(point)
            elif i == 4:
                self.publisher_drone4.publish(point)
            self.get_logger().info('Published computed point %d to /drone%d/new_goal: x=%f, y=%f, z=%f' %
                                   (i, i, point.x, point.y, point.z))


def main(args=None):
    rclpy.init(args=args)
    straight_line_calculator = StraightLineCalculator()
    rclpy.spin(straight_line_calculator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
