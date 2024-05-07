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

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <occupancyMap/OccupancyGridMapRosConverter.h>
#include "fake_laser/LaserGenerator.h"
#include "ros_utils/TransformHandler.h"
#include "ros_utils/transform.h"
#include "ros_utils/pointcloud.h"

class FakeLaser
{
public:
  FakeLaser();

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

  void updateLaserPose(const ros::TimerEvent& event);

  void generateFakeLaser(const ros::TimerEvent& event);

public:
  ros::NodeHandle nh_priv_{ "~" };
  ros::NodeHandle nh_{ "fake_odom_node" };
  utils::TransformHandler tf_;

  // Topics
  std::string map_topic_{ nh_priv_.param<std::string>("map", "/map") };
  std::string laser_topic_{ nh_priv_.param<std::string>("laser", "laser") };

  // Frame Ids
  std::string baselink_frame{ nh_.param<std::string>("baselinkFrame", "base_link") };
  std::string odom_frame{ nh_.param<std::string>("odometryFrame", "odom") };
  std::string map_frame{ nh_.param<std::string>("mapFrame", "map") };
  std::string laser_frame{ nh_priv_.param<std::string>("laserFrame", "laser") };

  // Timer Options
  double pose_update_rate_{ nh_priv_.param<double>("poseUpdateRate", 20.0) };
  double laser_pub_rate_{ nh_priv_.param<double>("laserscanPubRate", 20.0) };

  // Fake Laser Parameters
  double laserRange_min_{ nh_priv_.param<double>("laser_range_min", 0.05) };
  double laserRange_max_{ nh_priv_.param<double>("laser_range_max", 10.0) };
  double laserOffset_x_{ nh_priv_.param<double>("laser_offset_x", 0) };
  double laserOffset_y_{ nh_priv_.param<double>("laser_offset_y", 0) };
  double laserOffset_z_{ nh_priv_.param<double>("laser_offset_z", 0.5) };

  // ROS
  ros::Subscriber sub_map_{ nh_priv_.subscribe(map_topic_, 1, &FakeLaser::mapCallback, this) };
  ros::Publisher pub_laser_{ nh_priv_.advertise<sensor_msgs::PointCloud2>(laser_topic_, 1) };

  ros::Timer pose_update_timer_{ nh_priv_.createTimer(pose_update_rate_, &FakeLaser::updateLaserPose, this) };
  ros::Timer pub_laser_timer_{ nh_priv_.createTimer(laser_pub_rate_, &FakeLaser::generateFakeLaser, this, false,
                                                    false) };

private:
  OccupancyGridMap occupancy_map_;
  LaserGenerator laser_generator_;
};

#endif