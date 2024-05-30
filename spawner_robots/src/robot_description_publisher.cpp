#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

class RobotDescriptionPublisher : public rclcpp::Node
{
public:
  RobotDescriptionPublisher(const std::vector<std::string> & args)
    : Node("robot_description_pub")
  {
    // Command-line argument parsing
    if (args.size() == 1) {
      throw std::runtime_error("Recibed xml, publishing in robot_description topic");
    }
    std::string xml_string = args[1];

    
    // RCLCPP_INFO(get_logger(), "Topic to publish ==> %s", robot_description_topic.c_str());

    // Create the publisher                               
    rclcpp::QoS qos = rclcpp::QoS(100000);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    publisher_ = create_publisher<std_msgs::msg::String>("robot_description", qos);

    // Send data
    send(xml_string);
  }

private:
  void send(const std::string & xml_data)
  {
    auto message = std::make_unique<std_msgs::msg::String>();
    message->data = xml_data;
    publisher_->publish(std::move(message));
    RCLCPP_INFO(get_logger(), "Publishing XML DATA... DONE");
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  while(1){
    rclcpp::spin(std::make_shared<RobotDescriptionPublisher>(args));
  }
  rclcpp::shutdown();
  return 0;
}
