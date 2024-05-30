#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist twist_msg;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    // Split the received string by comma to separate linear and angular velocities
    std::istringstream iss(msg->data);
    std::string linear_str, angular_str;
    getline(iss, linear_str, ',');
    getline(iss, angular_str);

    double linear_vel, angular_vel;

    try
    {
        linear_vel = std::stod(linear_str);
        angular_vel = std::stod(angular_str);

    }
    catch (const std::invalid_argument& e)
    {
        ROS_ERROR("Invalid velocity values");
        return;
    }
    
     // Create a Twist message and fill in the linear and angular velocities
    
    twist_msg.linear.x = linear_vel;
    twist_msg.angular.z = angular_vel;    
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "traductor");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("str_cmd_vel", 1000, chatterCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  ros::Rate rate(100);
  while(ros::ok()){
    // Publish the Twist message to the /cmd_vel topic
    if (twist_msg.linear.x != 0.0 || twist_msg.angular.z != 0.0){
      pub.publish(twist_msg);
    }
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}