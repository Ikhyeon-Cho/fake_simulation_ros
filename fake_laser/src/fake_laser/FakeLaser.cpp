/*
 * FakeLaser.cpp
 *
 *  Created on: Dec 8, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fake_laser/FakeLaser.h"

FakeLaser::FakeLaser()
{
  // static transform publisher in here: laser2Base

  laser_generator_.setMinRange(laserRange_min_);
  laser_generator_.setMaxRange(laserRange_max_);
}

void FakeLaser::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (OccupancyGridMapRosConverter::fromOccupancyGridMsg(*msg, occupancy_map_))
    sub_map_.shutdown();

  pub_laser_timer_.start();
}

void FakeLaser::updateLaserPose(const ros::TimerEvent& event)
{
  // TODO: make staicTransform Broadcaster
  geometry_msgs::Transform laser2Base;
  laser2Base.translation.x = laserOffset_x_;
  laser2Base.translation.y = laserOffset_y_;
  laser2Base.translation.z = laserOffset_z_;
  laser2Base.rotation = utils::tf::getQuaternionMsgFrom(0, 0, 0);

  tf_.sendTransform(laser2Base, baselink_frame, laser_frame, ros::Time::now());

  auto [has_transform_l2m, laser2Map] = tf_.getTransform(laser_frame, map_frame);
  if (!has_transform_l2m)
    return;

  const auto& laser_pose_x = laser2Map.transform.translation.x;
  const auto& laser_pose_y = laser2Map.transform.translation.y;
  const auto& laser_height = laser2Map.transform.translation.z;

  Eigen::Vector2d laserPosition_in_map(laser_pose_x, laser_pose_y);
  laser_generator_.setLaserOrigin(laserPosition_in_map, laser_height);
}

void FakeLaser::generateFakeLaser(const ros::TimerEvent& event)
{
  auto ray_laser = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  ray_laser->header.frame_id = map_frame;
  if (!laser_generator_.doRayCasting(occupancy_map_, *ray_laser))
  {
    ROS_WARN_THROTTLE(10, "Failed to generate fake laser scan. Laser is out of map.\n");
    return;
  }

  auto [has_transform_m2l, map2laser] = tf_.getTransform(map_frame, laser_frame);
  if (!has_transform_m2l)
    return;

  auto fake_laser_raw = utils::pcl::transformPointcloud<pcl::PointXYZ>(ray_laser, map2laser);

  // filter pointcloud
  auto fake_laser_range_filtered = utils::pcl::filterPointcloudByRange2D<pcl::PointXYZ>(
      fake_laser_raw, laser_generator_.getMinRange(), laser_generator_.getMaxRange());

  // Typical 2D LiDAR specification: -135 ~ 135 degree
  auto fake_laser_angle_filtered =
      utils::pcl::filterPointcloudByAngle<pcl::PointXYZ>(fake_laser_range_filtered, -135, 135);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*fake_laser_angle_filtered, msg);
  pub_laser_.publish(msg);
}