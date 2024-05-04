/*
 * FakeOdometry.cpp
 *
 *  Created on: Dec 7, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fake_odom/FakeOdometry.h"

void FakeOdometry::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
  cmd_vel_ = *msg;
}

void FakeOdometry::setInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  geometry_msgs::PoseWithCovarianceStamped initialpose_in_odom;
  auto [has_transform, transform] = tf_.getTransform(msg->header.frame_id, odom_frame);
  if (!has_transform || !utils::tf::doTransform(*msg, initialpose_in_odom, transform))
  {
    std::cout << "Temp: initialpose to initialpose? transform is not valid. skip setting initial pose \n";
    return;
  }

  // set Pose
  auto [roll, pitch, initial_pose_yaw] = utils::tf::getRPYFrom(initialpose_in_odom.pose.pose.orientation);
  const auto& initial_pose_x = initialpose_in_odom.pose.pose.position.x;
  const auto& initial_pose_y = initialpose_in_odom.pose.pose.position.y;
  robot_.setPose2D(Eigen::Vector3d(initial_pose_x, initial_pose_y, initial_pose_yaw));

  // reset velocty as zero
  cmd_vel_ = geometry_msgs::Twist();
}

void FakeOdometry::deadReckoning(const ros::TimerEvent& event)
{
  const auto& v = cmd_vel_.linear.x;
  const auto& w = cmd_vel_.angular.z;
  Eigen::Vector2d cmd_vel(v, w);

  robot_.deadReckoning(cmd_vel, 1 / odom_pub_rate_);
  const auto& pose_2d = robot_.getPose2D();

  // Publish odom and tf
  nav_msgs::Odometry msg_odom;
  toOdomMsg(pose_2d, msg_odom);
  pub_odom_.publish(msg_odom);

  geometry_msgs::Transform transform;
  toTransformMsg(pose_2d, transform);
  tf_.sendTransform(transform, odom_frame, baselink_frame, ros::Time::now());

  if (enable_map2odom_)
  {
    geometry_msgs::Transform transform;
    toTransformMsg(Eigen::Vector3d(0, 0, 0), transform);
    tf_.sendTransform(transform, map_frame, odom_frame, ros::Time::now());
  }
}

void FakeOdometry::toOdomMsg(const Eigen::Vector3d& pose2D, nav_msgs::Odometry& msg)
{
  msg.header.frame_id = odom_frame;
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = baselink_frame;

  msg.pose.pose.position.x = pose2D.x();
  msg.pose.pose.position.y = pose2D.y();
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation = utils::tf::getQuaternionMsgFrom(0, 0, pose2D.z());

  // velocity
  msg.twist.twist = cmd_vel_;
}

void FakeOdometry::toTransformMsg(const Eigen::Vector3d& pose2D, geometry_msgs::Transform& transform)
{
  transform.translation.x = pose2D.x();
  transform.translation.y = pose2D.y();

  auto rotation = utils::tf::getQuaternionMsgFrom(0, 0, pose2D.z());
  transform.rotation = rotation;
}