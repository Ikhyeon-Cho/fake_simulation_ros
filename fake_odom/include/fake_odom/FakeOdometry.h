/*
 * FakeOdometry.h
 *
 *  Created on: Dec 7, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FAKE_ODOM_ROS_H
#define FAKE_ODOM_ROS_H

#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>
#include "fake_odom/Robot.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

namespace ros
{
class FakeOdometry
{
public:
  FakeOdometry();

  void saveControlInput(const geometry_msgs::TwistConstPtr& msg);

  void doDeadReckoning(const ros::TimerEvent& event);

  void setInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

public:
  // Subscribed Topics
  roscpp::Parameter<std::string> controlInput_topic{ "~/Subscribed_Topics/control_input", "/cmd_vel" };
  roscpp::Parameter<std::string> manualPose_topic{ "~/Subscribed_Topics/2d_pose_estimate", "/initialpose" };

  // Published Topics
  roscpp::Parameter<std::string> odom_topic{ "~/Published_Topics/odometry", "/odom_fake" };

  // Parameters
  // -- Framd Ids
  roscpp::Parameter<std::string> base_frameId{ "~/Parameters/base_frameId", "base_link" };
  roscpp::Parameter<std::string> odom_frameId{ "~/Parameters/odom_frameId", "odom" };
  roscpp::Parameter<std::string> map_frameId{ "~/Parameters/map_frameId", "map" };
  // -- Duration
  roscpp::Parameter<double> odom_publish_duration{ "~/Parameters/odom_publish_duration", 0.02 };
  // -- Options
  roscpp::Parameter<bool> use_map2odom_broadcaster{"~/Parameters/use_map2odom_broadcaster", true};

private:
  void toOdomMsg(const Eigen::Vector3d& pose2D, nav_msgs::Odometry& msg);

  void toTransformMsg(const Eigen::Vector3d& pose2D, geometry_msgs::Transform& transform);

private:
  Robot robot_;
  ros::TransformHandler transform_handler_;
  geometry_msgs::Twist control_input_msg_;

  roscpp::Subscriber<geometry_msgs::Twist> controlInput_subscriber{ controlInput_topic.param(),
                                                                    &FakeOdometry::saveControlInput, this };
  roscpp::Subscriber<geometry_msgs::PoseWithCovarianceStamped> manualPose_subscriber{ manualPose_topic.param(),
                                                                                      &FakeOdometry::setInitialPose,
                                                                                      this };

  roscpp::Publisher<nav_msgs::Odometry> odom_publisher{ odom_topic.param() };
  roscpp::Timer deadReckoning_timer{ odom_publish_duration.param(), &FakeOdometry::doDeadReckoning, this };
};

}  // namespace ros

#endif