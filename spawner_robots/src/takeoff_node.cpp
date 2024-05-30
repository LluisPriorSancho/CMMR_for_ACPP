#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include "spawner_robots/srv/command.hpp"
#include "tello_msgs/msg/tello_response.hpp"

#include <thread>

class DronesTakeoff : public rclcpp::Node
{
public:
  DronesTakeoff()
    : Node("DronesTakeoff")
  {
    // Create the service for taking off all drones
    command_all_service = create_service<spawner_robots::srv::Command>(
      "/command_all_drones",
      std::bind(&DronesTakeoff::commandAllDrones, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create subscriptions to tello_response topics for each drone
    subscription_drone1 = create_subscription<tello_msgs::msg::TelloResponse>(
      "/drone1/tello_response",
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DronesTakeoff::telloResponseCallback_drone1, this, std::placeholders::_1)
    );
    /*
    subscription_drone2 = create_subscription<tello_msgs::msg::TelloResponse>(
      "/drone2/tello_response",
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DronesTakeoff::telloResponseCallback_drone2, this, std::placeholders::_1)
    );

    subscription_drone3 = create_subscription<tello_msgs::msg::TelloResponse>(
      "/drone3/tello_response",
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DronesTakeoff::telloResponseCallback_drone3, this, std::placeholders::_1)
    );

    subscription_drone4 = create_subscription<tello_msgs::msg::TelloResponse>(
      "/drone4/tello_response",
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DronesTakeoff::telloResponseCallback_drone4, this, std::placeholders::_1)
    );*/

    // Initialize flags
    flag_drone1 = false;
    flag_drone2 = false;
    flag_drone3 = false;
    flag_drone4 = false;

    // Start threads for service and subscriptions
    service_thread = std::thread(&DronesTakeoff::serviceThreadFunc, this);
    subscription_thread = std::thread(&DronesTakeoff::subscriptionThreadFunc, this);
  }

  ~DronesTakeoff()
  {
    // Join threads to avoid undefined behavior
    if (service_thread.joinable()) {
      service_thread.join();
    }
    if (subscription_thread.joinable()) {
      subscription_thread.join();
    }
  }

private:
  void callService(const std::string& service_name, const std::string& command)
  {
    auto client = create_client<tello_msgs::srv::TelloAction>(service_name);

    while (!client->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "Service %s not available. Waiting...", service_name.c_str());
    }

    auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
    request->cmd = command;

    // Send the request
    client->async_send_request(request, [this, service_name](rclcpp::Client<tello_msgs::srv::TelloAction>::SharedFuture future_result) {
      // Callback function when the response is received
      auto result = future_result.get();
      if (result->rc == 1) {
        RCLCPP_INFO(get_logger(), "Drone on %s is ready to receive commands.", service_name.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to execute command on drone %s.", service_name.c_str());
      }
    });
  }

  void commandAllDrones(
    const std::shared_ptr<spawner_robots::srv::Command::Request> request,
    std::shared_ptr<spawner_robots::srv::Command::Response> response)
  {
    auto command_value = request->order;

    // Call the service for each drone
    for (int drone_id = 1; drone_id <= 4; ++drone_id) {
      std::string service_name = "/drone" + std::to_string(drone_id) + "/tello_action";
      callService(service_name, command_value);
    }

    if (flag_drone1 && flag_drone2 && flag_drone3 && flag_drone4){
      response->status = true;
    }
    else{
      response->status = false;
    }
  }

  void telloResponseCallback_drone1(const tello_msgs::msg::TelloResponse::SharedPtr msg)
  {
    auto trash = msg; // To avoid unused variable warning
    flag_drone1 = true; 
    RCLCPP_INFO(get_logger(), "Received tello_response from drone1");
  }
  void telloResponseCallback_drone2(const tello_msgs::msg::TelloResponse::SharedPtr msg)
  {
    auto trash = msg; // To avoid unused variable warning
    flag_drone2 = true; 
    RCLCPP_INFO(get_logger(), "Received tello_response from drone2");
  }
  void telloResponseCallback_drone3(const tello_msgs::msg::TelloResponse::SharedPtr msg)
  {
    auto trash = msg; // To avoid unused variable warning
    flag_drone3 = true; 
    RCLCPP_INFO(get_logger(), "Received tello_response from drone3");
  }
  void telloResponseCallback_drone4(const tello_msgs::msg::TelloResponse::SharedPtr msg)
  {
    auto trash = msg; // To avoid unused variable warning
    flag_drone4 = true; 
    RCLCPP_INFO(get_logger(), "Received tello_response from drone4");
  }
  void serviceThreadFunc()
  {
    rclcpp::spin(std::make_shared<rclcpp::Node>("service_thread"));
  }

  void subscriptionThreadFunc()
  {
    rclcpp::spin(std::make_shared<rclcpp::Node>("subscription_thread"));
  }

  rclcpp::Service<spawner_robots::srv::Command>::SharedPtr command_all_service;
  rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr subscription_drone1;
  rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr subscription_drone2;
  rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr subscription_drone3;
  rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr subscription_drone4;

  std::thread service_thread;
  std::thread subscription_thread;

  bool flag_drone1;
  bool flag_drone2;
  bool flag_drone3;
  bool flag_drone4;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<DronesTakeoff>();

  // Spin the node
  rclcpp::spin(node);

  rclcpp::shutdown();
  
  return 0;
}
