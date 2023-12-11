/*
 * FakeLaser.h
 *
 *  Created on: Dec 8, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FAKE_LASER_ROS_H
#define FAKE_LASER_ROS_H

#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>
#include <ros_pcl_utils/PointcloudProcessor.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <occupancy_grid_map/OccupancyGridMapRosConverter.h>
#include "fake_laser/LaserGenerator.h"

namespace ros
{
class FakeLaser
{
public:
  FakeLaser();

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

  void updateLaserPose(const ros::TimerEvent& event);

  void generateFakeLaser(const ros::TimerEvent& event);

public:
  // Subscribed Topics
  roscpp::Parameter<std::string> map_topic{ "~/Subscribed_Topics/map", "map" };

  // Published Topics
  roscpp::Parameter<std::string> laser_topic{ "~/Published_Topics/laser", "laser" };

  // Parameters
  // -- Frame Ids
  roscpp::Parameter<std::string> base_frameId{ "~/Parameters/base_frameId", "base_link" };
  roscpp::Parameter<std::string> odom_frameId{ "~/Parameters/odom_frameId", "odom" };
  roscpp::Parameter<std::string> map_frameId{ "~/Parameters/map_frameId", "map" };
  roscpp::Parameter<std::string> laser_frameId{ "~/Parameters/laser_frameId", "laser" };
  // -- Duration
  roscpp::Parameter<double> poseUpdate_duration{ "~/Parameters/pose_update_duration", 0.05 };
  roscpp::Parameter<double> laserPublish_duration{ "~/Parameters/laserscan_publish_duration", 0.05 };
  // -- laser param
  roscpp::Parameter<double> laserRange_min{ "~/Parameters/laser_range_min", 0.05 };
  roscpp::Parameter<double> laserRange_max{ "~/Parameters/laser_range_max", 10.0 };
  roscpp::Parameter<double> laserOffset_x{ "~/Parameters/laser_offset_x", 0 };
  roscpp::Parameter<double> laserOffset_y{ "~/Parameters/laser_offset_y", 0 };
  roscpp::Parameter<double> laserOffset_z{ "~/Parameters/laser_offset_z", 0.5 };

private:
  TransformHandler transform_handler_;
  PointcloudProcessor<pcl::PointXYZI> pointcloud_handler_;
  OccupancyGridMap occupancy_map_;
  LaserGenerator laser_generator_;

  roscpp::Subscriber<nav_msgs::OccupancyGrid> occupancy_map_subscriber{ map_topic.param(), &FakeLaser::mapCallback,
                                                                        this };
  roscpp::Publisher<sensor_msgs::PointCloud2> laser_scan_publisher{ laser_topic.param() };

  roscpp::Timer pose_update_timer{ poseUpdate_duration, &FakeLaser::updateLaserPose, this };
  roscpp::Timer laser_generation_timer{ laserPublish_duration, &FakeLaser::generateFakeLaser, this };
};

}  // namespace ros

#endif