/*
 * LaserGenerator.cpp
 *
 *  Created on: Dec 8, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fake_laser/LaserGenerator.h"

LaserGenerator::LaserGenerator()
{
}

void LaserGenerator::setLaserOrigin(const Eigen::Vector2d& laser_position, double laser_height)
{
  laser_position_ = laser_position;
  laser_height_ = laser_height;
}

void LaserGenerator::setMaxRange(double range_max)
{
  range_max_ = range_max;
}

void LaserGenerator::setMinRange(double range_min)
{
  range_min_ = range_min;
}

double LaserGenerator::getMaxRange() const
{
  return range_max_;
}

double LaserGenerator::getMinRange() const
{
  return range_min_;
}

bool LaserGenerator::doRayCasting(const OccupancyGridMap& map, pcl::PointCloud<pcl::PointWithRange>& laser)
{
  // start at the laser
  grid_map::Index start_index;
  if (!map.getIndex(laser_position_, start_index))
  {
    // std::cout << "laser is out of map. \n"; 
    return false;
  }

  std::vector<grid_map::Index> end_index_list;
  for (double i = -range_max_; i < range_max_; i += map.getResolution())
  {
    grid_map::Index end_index;
    grid_map::Position end_position;
    // 1. (-range_max, i)
    end_position = laser_position_ + grid_map::Position(-range_max_, i);
    if (map.getIndex(end_position, end_index))
      end_index_list.push_back(end_index);
    else
    {
      end_position = map.getClosestPositionInMap(end_position);
      map.getIndex(end_position, end_index);
      end_index_list.push_back(end_index);
    }
    // 2. (range_max, i)
    end_position = laser_position_ + grid_map::Position(range_max_, i);
    if (map.getIndex(end_position, end_index))
      end_index_list.push_back(end_index);
    else
    {
      end_position = map.getClosestPositionInMap(end_position);
      map.getIndex(end_position, end_index);
      end_index_list.push_back(end_index);
    }
    // 3. (i, -range_max)
    end_position = laser_position_ + grid_map::Position(i, -range_max_);
    if (map.getIndex(end_position, end_index))
      end_index_list.push_back(end_index);
    else
    {
      end_position = map.getClosestPositionInMap(end_position);
      map.getIndex(end_position, end_index);
      end_index_list.push_back(end_index);
    }
    // 4. (i, range_max)
    end_position = laser_position_ + grid_map::Position(i, range_max_);
    if (map.getIndex(end_position, end_index))
      end_index_list.push_back(end_index);
    else
    {
      end_position = map.getClosestPositionInMap(end_position);
      map.getIndex(end_position, end_index);
      end_index_list.push_back(end_index);
    }
  }

  // ray tracing
  for (const auto& end_index : end_index_list)
  {
    for (grid_map::LineIterator iterator(map, start_index, end_index); !iterator.isPastEnd(); ++iterator)
    {
      if (std::abs(map.at("occupancy", *iterator)) < 1e-3)
        continue;

      pcl::PointWithRange point;
      auto position = map.getPositionFrom(*iterator);
      point.x = position.x();
      point.y = position.y();
      point.z = laser_height_;
      point.range = (position - laser_position_).norm();
      laser.points.push_back(point);
      break;
    }
  }

  return true;
}