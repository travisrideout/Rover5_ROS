/*
 * joy_teleop.h
 *
 *  Created on: Feb 22, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_JOY_TELEOP_H_
#define INCLUDE_ROVER5_ROS_JOY_TELEOP_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "Rover5_ROS/rover.h"

extern Rover5_ROS::rover rover_msg;

class Joy_TeleOp
{
public:
	Joy_TeleOp();
	virtual ~Joy_TeleOp();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int left_CMD, right_CMD;
  double l_scale_, a_scale_;
  //ros::Publisher vel_pub_;
  ros::Publisher rover_pub;
  ros::Subscriber joy_sub_;

};

#endif /* INCLUDE_ROVER5_ROS_JOY_TELEOP_H_ */
