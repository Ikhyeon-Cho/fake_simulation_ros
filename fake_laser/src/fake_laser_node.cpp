/*
 * fake_laser_node.cpp
 *
 *  Created on: Dec 8, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "fake_laser/FakeLaser.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_laser_node");
  ros::NodeHandle nh("~");

  ros::FakeLaser node;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}