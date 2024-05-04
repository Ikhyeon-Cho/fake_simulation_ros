/*
 * FakeOdometry.h
 *
 *  Created on: Dec 7, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FAKE_ODOMETRY_ROS_H
#define FAKE_ODOMETRY_ROS_H

#include "ros_utils/TransformHandler.h"
#include "ros_utils/transform.h"
#include "fake_odom/Robot.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

class FakeOdometry
{
public:
  FakeOdometry() = default;

  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

  void deadReckoning(const ros::TimerEvent& event);

  void setInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

public:
  ros::NodeHandle nh_priv_{ "~" };
  utils::TransformHandler tf_;

  // Topics
  std::string cmdvel_topic_{ nh_priv_.param<std::string>("cmd_vel", "/cmd_vel") };
  std::string initialpose_topic_{ nh_priv_.param<std::string>("initialpose", "/initialpose") };
  std::string odom_topic_{ nh_priv_.param<std::string>("odom", "/odom_fake") };

  // Framd Ids
  std::string baselink_frame{ nh_priv_.param<std::string>("baselinkFrame", "base_link") };
  std::string odom_frame{ nh_priv_.param<std::string>("odometryFrame", "odom") };
  std::string map_frame{ nh_priv_.param<std::string>("mapFrame", "map") };

  // Timer Options
  double odom_pub_rate_{ nh_priv_.param<double>("odometryPubRate", 50.0) };

  // TF Broadcaster Options
  bool enable_map2odom_{ nh_priv_.param<bool>("use_map2odom_broadcaster", true) };

  // ROS
  ros::Subscriber sub_cmdvel_{ nh_priv_.subscribe(cmdvel_topic_, 1, &FakeOdometry::cmdVelCallback, this) };
  ros::Subscriber sub_initialpose_{ nh_priv_.subscribe(initialpose_topic_, 1, &FakeOdometry::setInitialPose, this) };
  ros::Publisher pub_odom_{ nh_priv_.advertise<nav_msgs::Odometry>(odom_topic_, 1) };
  ros::Timer pub_odom_timer_{ nh_priv_.createTimer(odom_pub_rate_, &FakeOdometry::deadReckoning, this) };

private:
  void toOdomMsg(const Eigen::Vector3d& pose_2d, nav_msgs::Odometry& msg);

  void toTransformMsg(const Eigen::Vector3d& pose_2d, geometry_msgs::Transform& transform);

private:
  Robot robot_;
  geometry_msgs::Twist cmd_vel_;
};

#endif  // FAKE_ODOMETRY_ROS_H