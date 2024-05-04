/*
 * fake_odom_node.cpp
 *
 *  Created on: Dec 7, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "fake_odom/FakeOdometry.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_odom_node");
  ros::NodeHandle nh("~");

  FakeOdometry node;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}