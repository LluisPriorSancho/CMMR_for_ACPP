#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  tf::TransformListener listener;
  ros::Publisher tf_pub = node.advertise<std_msgs::String>("str_tf_listener", 10);

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    std_msgs::String msg;
    std::stringstream ss;
    ss << transform.getOrigin().x() << ","
       << transform.getOrigin().y() << ","
       << transform.getOrigin().z() << ","
       << transform.getRotation().x() << ","
       << transform.getRotation().y() << ","
       << transform.getRotation().z() << ","
       << transform.getRotation().w();
    msg.data = ss.str();
    tf_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

