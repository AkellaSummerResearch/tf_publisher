#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "arena_tf_publisher/transformations.hpp"
Eigen::Matrix3d ecef2enu_rotMatrix(Eigen::Vector3d ECEF);
Eigen::Vector3d ecef2enu(Eigen::Vector3d ECEF);

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
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg->header.frame_id, "base_link"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "arena_tf_broadcaster");
  ros::NodeHandle node("~");
  ROS_INFO("Tf broadcaster started!");

  // Get arena center so we publish it into tf
  Eigen::Vector3d baseECEF_vector;
  baseECEF_vector(0) = 0;
  baseECEF_vector(1) = 0;
  baseECEF_vector(2) = 0;
  Eigen::Matrix3d Recef2enu = ecef2enu_rotMatrix(baseECEF_vector);

  // Publish arena center
  // static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  // geometry_msgs::TransformStamped static_transformStamped;
  // TODO: Get center of arena in ECEF frame

  ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, poseCallback);

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