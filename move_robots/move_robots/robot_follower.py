import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from tf2_ros import Buffer, TransformListener
from spawner_robots.srv import Command
import time

class FollowerLineCalculator(Node):
    def __init__(self):
        super().__init__('straight_line_calculator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publishers for each drone
        self.publisher_drone1 = self.create_publisher(Vector3, '/drone1/new_goal', 10)
        self.publisher_drone2 = self.create_publisher(Vector3, '/drone2/new_goal', 10)
        self.publisher_drone3 = self.create_publisher(Vector3, '/drone3/new_goal', 10)
        self.publisher_drone4 = self.create_publisher(Vector3, '/drone4/new_goal', 10)

        # Create a client to call the service
        self.command_all_drones_client = self.create_client(Command, '/command_all_drones')

        # Wait for the service to be available
        while not self.command_all_drones_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service "/command_all_drones" not available. Waiting...')

        # Command all drones to take off
        result = False
        while (not(result)):
            result = self.call_command_all_drones('takeoff')

    # Call the service that takes off all drones
    def call_command_all_drones(self, order):
        request = Command.Request()
        request.order = order
        future = self.command_all_drones_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Command sent to all drones: %s' % order)
        else:
            self.get_logger().error('Failed to send command to all drones')
        time.sleep(1)
        result = future.result()
        return result.status


    def on_timer(self):
        self.compute_points()

    def compute_points(self):
        # Get the position of the rover on gazebo
        try:
            transform = self.tf_buffer.lookup_transform('map', 'rover1/base_footprint', rclpy.time.Time())
            position = transform.transform.translation
        except Exception as e:
            self.get_logger().error(f"Failed to get TF transform: {str(e)}")
            return

        # Compute points in a straight line between rover1/base_footprint and origin
        for i in range(1, 5):
            x = position.x / 5 * i
            y = position.y / 5 * i
            z = position.z / 5 * i
            point = Vector3(x=x, y=y, z=z)

            # Publish computed points to the respective drone's topic
            if i == 1:
                self.publisher_drone1.publish(point)
            elif i == 2:
                self.publisher_drone2.publish(point)
            elif i == 3:
                self.publisher_drone3.publish(point)
            elif i == 4:
                self.publisher_drone4.publish(point)
            #self.get_logger().info('Published computed point %d to /drone%d/new_goal: x=%f, y=%f, z=%f' %
            #                       (i, i, point.x, point.y, point.z))

def main(args=None):
    rclpy.init(args=args)
    follower_line_calculator = FollowerLineCalculator()
    timer_period = 0.1  # seconds
    follower_line_calculator.create_timer(timer_period, follower_line_calculator.on_timer)
    rclpy.spin(follower_line_calculator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
