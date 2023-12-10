/*
 * FakeLaser.cpp
 *
 *  Created on: Dec 8, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fake_laser/FakeLaser.h"

namespace ros
{
FakeLaser::FakeLaser()
{
  // static transform publisher in here: laser2Base

  laser_generator_.setMinRange(laserRange_min.param());
  laser_generator_.setMaxRange(laserRange_max.param());
  pose_update_timer.start();
}

void FakeLaser::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (OccupancyGridMapRosConverter::fromOccupancyGridMsg(*msg, occupancy_map_))
    occupancy_map_subscriber.shutdown();

  laser_generation_timer.start();
}

void FakeLaser::updateLaserPose(const ros::TimerEvent& event)
{
  // TODO: make staicTransform Broadcaster
  geometry_msgs::Transform laser2Base;
  laser2Base.translation.x = laserOffset_x.param();
  laser2Base.translation.y = laserOffset_y.param();
  laser2Base.translation.z = laserOffset_z.param();
  transform_handler_.getQuaternionFrom(0, 0, 0, laser2Base.rotation);
  transform_handler_.sendTransform(laser2Base, base_frameId.param(), laser_frameId.param(), ros::Time::now());

  geometry_msgs::TransformStamped laser2Map;
  if (!transform_handler_.getTransform(map_frameId.param(), laser_frameId.param(), laser2Map))
    return;

  const auto& laser_pose_x = laser2Map.transform.translation.x;
  const auto& laser_pose_y = laser2Map.transform.translation.y;
  Eigen::Vector2d laserPosition_in_map(laser_pose_x, laser_pose_y);
  
  const auto& laser_height = laser2Map.transform.translation.z;

  laser_generator_.setLaserOrigin(laserPosition_in_map, laser_height);
}

void FakeLaser::generateFakeLaser(const ros::TimerEvent& event)
{
  auto ray_laser = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  ray_laser->header.frame_id = map_frameId.param();
  laser_generator_.doRayCasting(occupancy_map_, *ray_laser);

  bool has_transformed_cloud(false);
  auto fake_laser_raw =
      pointcloud_handler_.transformPointcloud(ray_laser, laser_frameId.param(), has_transformed_cloud);
  if (!has_transformed_cloud)
    return;

  // filter pointcloud
  auto fake_laser_range_filtered = pointcloud_handler_.filterPointcloudByRange2D(
      fake_laser_raw, laser_generator_.getMinRange(), laser_generator_.getMaxRange());

  auto fake_laser_angle_filtered = pointcloud_handler_.filterPointcloudByAngle(fake_laser_range_filtered, -135, 135);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*fake_laser_angle_filtered, msg);
  laser_scan_publisher.publish(msg);
}

}  // namespace ros