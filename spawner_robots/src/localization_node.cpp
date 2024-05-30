#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"

class LocationNode : public rclcpp::Node
{
public:
  LocationNode()
    : Node("location_node")
  {
    // Create threads for publishing and making service calls
    publish_thread_ = std::thread(std::bind(&LocationNode::publishThread, this));
    call_thread_ = std::thread(std::bind(&LocationNode::callThread, this));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);    
  }

private:
  void publishThread()
  {
    //rclcpp::Rate rate(1);  // 10 Hz
    while (rclcpp::ok()) {
      // Access the stored poses and print them
      std::lock_guard<std::mutex> lock(tf_mutex_);
      for (const auto& data : tf_data_) {
        auto pose = data.second;
        /*RCLCPP_INFO(get_logger(), "Entity: %s, Pose (x, y, z): %.2f, %.2f, %.2f",
                    data.first.c_str(), pose.position.x, pose.position.y, pose.position.z);*/
      
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "map";

        // Set the child_frame_id based on the entity
        if (data.first == "tello_1") {
            tf_msg.child_frame_id = "base_link_1";
        } else if (data.first == "tello_2") {
            tf_msg.child_frame_id = "base_link_2";
        } else if (data.first == "tello_3") {
            tf_msg.child_frame_id = "base_link_3";
        } else if (data.first == "tello_4") {
            tf_msg.child_frame_id = "base_link_4";
        } else if (data.first == "leo1") {
            tf_msg.child_frame_id = "rover1/base_footprint";
        }
        
        tf_msg.transform.translation.x = pose.position.x;
        tf_msg.transform.translation.y = pose.position.y;
        tf_msg.transform.translation.z = pose.position.z;
        tf_msg.transform.rotation = pose.orientation;

        tf_broadcaster_->sendTransform(tf_msg);
      }

      // Sleep to achieve the desired rate
      //rate.sleep();
    }
  }


  void callThread()
  {
    // Set up the service client
    client_ = create_client<gazebo_msgs::srv::GetEntityState>("/demo/get_entity_state");
    
    pub_leo1_position = create_publisher<geometry_msgs::msg::Point>("/leo1_position", 10);
    pub_tello1_position = create_publisher<geometry_msgs::msg::Point>("/tello1_position", 10);

    while (rclcpp::ok()) {
      
      // Iterate through entities and make service calls
      for (const auto& entity : entities_) {
        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = entity;
        request->reference_frame = "world";

        auto future_result = client_->async_send_request(request);

        // Wait for the result with a timeout
        if (future_result.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
          auto result = future_result.get();

          // Lock the TF data mutex
          std::lock_guard<std::mutex> lock(tf_mutex_);

          // Update TF data
          tf_data_[entity] = result->state.pose;

          // Publish the position data of the leo and of the first tello drone
          if (entity == "leo1"){
            auto msg_leo1 = geometry_msgs::msg::Point();
            msg_leo1.x = result->state.pose.position.x;
            msg_leo1.y = result->state.pose.position.y;
            msg_leo1.z = result->state.pose.position.z;
            pub_leo1_position->publish(msg_leo1);
          }

          if (entity == "tello_1"){
            auto msg_tello1 = geometry_msgs::msg::Point();
            msg_tello1.x = result->state.pose.position.x;
            msg_tello1.y = result->state.pose.position.y;
            msg_tello1.z = result->state.pose.position.z;
            pub_tello1_position->publish(msg_tello1);
          }

        } else {
          RCLCPP_ERROR(get_logger(), "Service call to /demo/get_entity_state failed.");
        }        
      }
    }
  }

  std::thread publish_thread_;
  std::thread call_thread_;
  std::vector<std::string> entities_ = {"tello_1", "leo1"};
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_;
  std::mutex tf_mutex_;
  std::map<std::string, geometry_msgs::msg::Pose> tf_data_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_leo1_position;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_tello1_position;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocationNode>());
  rclcpp::shutdown();
  return 0;
}
