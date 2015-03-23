/*
 * joy_teleop.cpp
 *
 *  Created on: Feb 22, 2015
 *      Author: travis
 */
#include <Rover5_ROS/joy_teleop.h>

Joy_TeleOp::Joy_TeleOp():left_CMD(1),right_CMD(4),width(0.01905){
	nh_.param("axis_left", left_CMD, left_CMD);
	nh_.param("axis_right", right_CMD, right_CMD);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	vel_pub = nh_.advertise<geometry_msgs::TwistWithCovariance>("rover_cmd_vel", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_TeleOp::joyCallback, this);
	std::cout << "Finished init of Joy_Teleop" << std::endl;
}

void Joy_TeleOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::TwistWithCovariance twist;
	twist.twist.linear.x = (joy->axes[left_CMD] + joy->axes[right_CMD])/2;
	twist.twist.angular.z = (joy->axes[right_CMD] - joy->axes[left_CMD])/width;		//right hand rule: counter clock wise is positive
	vel_pub.publish(twist);
}

Joy_TeleOp::~Joy_TeleOp() {
	std::cout << "Joy_TeleOp object terminated" << std::endl;
}


