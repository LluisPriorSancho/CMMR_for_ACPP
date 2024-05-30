#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RandomPublisher : public rclcpp::Node
{
public:
  RandomPublisher()
    : Node("random_publisher")
  {
    // Create publishers for each topic
    pub_drone1_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/drone1/cmd_vel", 10);
    pub_drone2_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/drone2/cmd_vel", 10);
    pub_drone3_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/drone3/cmd_vel", 10);
    pub_drone4_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/drone4/cmd_vel", 10);
    pub_leo1_cmd_unst_ = create_publisher<geometry_msgs::msg::Twist>("/leo1_velocity_controller/cmd_vel_unstamped", 10);

    // Create a timer to publish random values
    timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RandomPublisher::publishRandomValues, this));
  }

private:
  void publishRandomValues()
  {
    // Generate random values
    auto random_orientation = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
    auto random_velocity = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);

    // Create Twist messages with random values
    auto msg_drone1 = geometry_msgs::msg::Twist();
    msg_drone1.angular.z = random_orientation;
    msg_drone1.linear.x = random_velocity;
    pub_drone1_cmd_vel_->publish(msg_drone1);

    auto msg_drone2 = geometry_msgs::msg::Twist();
    msg_drone2.angular.z = random_orientation;
    msg_drone2.linear.x = random_velocity;
    pub_drone2_cmd_vel_->publish(msg_drone2);

    auto msg_drone3 = geometry_msgs::msg::Twist();
    msg_drone3.angular.z = random_orientation;
    msg_drone3.linear.x = random_velocity;
    pub_drone3_cmd_vel_->publish(msg_drone3);

    auto msg_drone4 = geometry_msgs::msg::Twist();
    msg_drone4.angular.z = random_orientation;
    msg_drone4.linear.x = random_velocity;
    pub_drone4_cmd_vel_->publish(msg_drone4);

    auto msg_leo1 = geometry_msgs::msg::Twist();
    msg_leo1.angular.z = random_orientation;
    msg_leo1.linear.x = random_velocity;
    pub_leo1_cmd_unst_->publish(msg_leo1);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_drone1_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_drone2_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_drone3_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_drone4_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_leo1_cmd_unst_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomPublisher>());
  rclcpp::shutdown();
  return 0;
}