#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include "std_msgs/String.h"

std::string ns;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  // ROS_INFO("Publishing into frame: %s", msg->header.frame_id.c_str());
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y, 
                                   msg->pose.pose.position.z));
  tf::Quaternion q(msg->pose.pose.orientation.x,
                   msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg->header.frame_id, ns + "/base_link"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "arena_tf_broadcaster");
  ros::NodeHandle node("~");
  ROS_INFO("Tf broadcaster started!");
  node.getParam("namespace", ns);

  // ns = ros::this_node::getNamespace();
  std::string topic = "/" + ns + "/mavros/local_position/odom";
  ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>(topic, 1, poseCallback);
  ROS_INFO("Tf broadcaster listening to: %s", sub.getTopic().c_str());

  // Get all quad names and create a subscriber for each one of them
  // std::vector<std::string> quadNames;
  // node.getParam("QuadList", quadNames);
  // std::vector<ros::Subscriber> subs;
  // for(int i =0; i < quadNames.size(); i++)
  // {
  //   std::string topic_name;
  //   topic_name = "/" + quadNames[i] + "/odom";
  //   ROS_INFO("[tf publisher]: Subscribing to: %s", topic_name.c_str());
  //   subs.push_back(node.subscribe(topic_name, 10, &poseCallback));
  // }

  ros::spin();
  return 0;
};