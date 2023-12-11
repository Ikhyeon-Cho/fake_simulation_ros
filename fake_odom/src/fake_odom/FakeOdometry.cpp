/*
 * FakeOdometry.cpp
 *
 *  Created on: Dec 7, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fake_odom/FakeOdometry.h"

namespace ros
{
FakeOdometry::FakeOdometry()
{
  deadReckoning_timer.start();
}

void FakeOdometry::saveControlInput(const geometry_msgs::TwistConstPtr& msg)
{
  control_input_msg_ = *msg;
}

void FakeOdometry::setInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  geometry_msgs::PoseWithCovarianceStamped pose_in_odom;
  if (!transform_handler_.doTransform(*msg, odom_frameId.param(), pose_in_odom))
  {
    std::cout << "Temp: map to odom? transform is not valid. skip setting initial pose \n";
    return;
  }

  // set Pose
  double roll, pitch, yaw;
  transform_handler_.getRPYFrom(pose_in_odom.pose.pose.orientation, roll, pitch, yaw);
  robot_.setPose2D(Eigen::Vector3d(pose_in_odom.pose.pose.position.x, pose_in_odom.pose.pose.position.y, yaw));

  // reset velocty as zero
  control_input_msg_ = geometry_msgs::Twist();
}

void FakeOdometry::doDeadReckoning(const ros::TimerEvent& event)
{
  const auto& v = control_input_msg_.linear.x;
  const auto& w = control_input_msg_.angular.z;
  Eigen::Vector2d velocity(v, w);

  robot_.doDeadReckoning(velocity, odom_publish_duration.param());
  const auto& current_pose2D = robot_.getPose2D();

  // Publish odom and tf
  nav_msgs::Odometry msg_odom;
  toOdomMsg(current_pose2D, msg_odom);
  odom_publisher.publish(msg_odom);

  ros::Time timestamp(ros::Time::now());
  geometry_msgs::Transform transform;
  toTransformMsg(current_pose2D, transform);
  transform_handler_.sendTransform(transform, odom_frameId.param(), base_frameId.param(), timestamp);

  if (use_map2odom_broadcaster.param())
  {
    geometry_msgs::Transform transform;
    toTransformMsg(Eigen::Vector3d(0, 0, 0), transform);
    transform_handler_.sendTransform(transform, map_frameId.param(), odom_frameId.param(), timestamp);
  }
}

void FakeOdometry::toOdomMsg(const Eigen::Vector3d& pose2D, nav_msgs::Odometry& msg)
{
  msg.header.frame_id = odom_frameId.param();
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = base_frameId.param();

  // position
  msg.pose.pose.position.x = pose2D.x();
  msg.pose.pose.position.y = pose2D.y();
  msg.pose.pose.position.z = 0;

  // orientation
  transform_handler_.getQuaternionFrom(0, 0, pose2D.z(), msg.pose.pose.orientation);

  // velocity
  msg.twist.twist = control_input_msg_;
}

void FakeOdometry::toTransformMsg(const Eigen::Vector3d& pose2D, geometry_msgs::Transform& transform)
{
  transform.translation.x = pose2D.x();
  transform.translation.y = pose2D.y();

  geometry_msgs::Quaternion rotation;
  transform_handler_.getQuaternionFrom(0, 0, pose2D.z(), rotation);
  transform.rotation = rotation;
}

}  // namespace ros