/*
 * Robot.h
 *
 *  Created on: Dec 7, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FAKE_ODOM_ROBOT_H
#define FAKE_ODOM_ROBOT_H

#include <Eigen/Core>

class Robot
{
public:
  Robot() = default;
  void setPose2D(const Eigen::Vector3d& pose2D)
  {
    pose2D_ = pose2D;
  }
  const Eigen::Vector3d& getPose2D() const
  {
    return pose2D_;
  }

  void doDeadReckoning(const Eigen::Vector2d& velocity, double dt)
  {
    const auto& v = velocity.x();
    const auto& w = velocity.y();

    Eigen::Vector3d delta_pose;
    delta_pose.x() = v * dt * std::cos(pose2D_.z());
    delta_pose.y() = v * dt * std::sin(pose2D_.z());
    delta_pose.z() = w * dt;

    pose2D_ += delta_pose;
  }

private:
  Eigen::Vector3d pose2D_{ 0, 0, 0 };
};

#endif