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
#include <geometry_msgs/TwistWithCovariance.h>

#define JOY_NUM_AXES			11
#define JOY_NUM_BUTTONS			8

#define T_X_AXIS				0
#define T_Y_AXIS				1
#define T_Z_AXIS				2
#define R_X_AXIS				3
#define R_Y_AXIS				4
#define R_Z_AXIS				5

#define BTN_BACK_SEL			0
#define BTN_START				1
#define BTN_X					2
#define BTN_Y					3
#define BTN_A					4
#define BTN_B					5
#define BTN_UP					6
#define BTN_DOWN				7
#define BTN_LEFT				8
#define BTN_RIGHT				9
#define BTN_LEFT_BACK			10
#define BTN_RIGHT_BACK			11
#define BTN_LEFT_THUMB			12
#define BTN_RIGHT_THUMB			13

//#define USE_SIXAXIS_CONTROLLER
#define USE_XBOX_CONTROLLER

#define GENERIC_JOY_NUM_AXES		9
#define GENERIC_JOY_NUM_BUTTONS		24

#define	width	0.1905

class Joy_TeleOp
{
public:
	Joy_TeleOp();
	virtual ~Joy_TeleOp();

private:
  void JoyCallback(const sensor_msgs::Joy& unordered_joy);
  void SetTwist();

  ros::NodeHandle nh_;

  //Publishers
  ros::Publisher vel_pub;

  //Subscribers
  ros::Subscriber joy_sub_;

  sensor_msgs::Joy joy_msg;

};

#endif /* INCLUDE_ROVER5_ROS_JOY_TELEOP_H_ */
