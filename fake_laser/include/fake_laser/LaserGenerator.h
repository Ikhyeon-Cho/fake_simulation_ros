/*
 * LaserGenerator.h
 *
 *  Created on: Dec 8, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef LASER_GENERATOR_H
#define LASER_GENERATOR_H

#include <occupancyMap/OccupancyGridMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LaserGenerator
{
public:
  LaserGenerator();

  void setLaserOrigin(const Eigen::Vector2d& laser_position, double laser_height);

  void setMaxRange(double range_max);
  void setMinRange(double range_min);

  double getMaxRange() const;
  double getMinRange() const;

  void doRayCasting(const OccupancyGridMap& map, pcl::PointCloud<pcl::PointXYZI>& laser);

private:
  Eigen::Vector2d laser_position_{ 0, 0 };
  double laser_height_{ 0.0 };
  double range_min_{ 0.01 };
  double range_max_{ 10.0 };
};

#endif