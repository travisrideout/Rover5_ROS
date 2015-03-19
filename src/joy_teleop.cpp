/*
 * joy_teleop.cpp
 *
 *  Created on: Feb 22, 2015
 *      Author: travis
 */
#include <Rover5_ROS/joy_teleop.h>

Joy_TeleOp::Joy_TeleOp():left_CMD(1),right_CMD(4){
	nh_.param("axis_left", left_CMD, left_CMD);
	nh_.param("axis_right", right_CMD, right_CMD);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	//vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	rover_pub = nh_.advertise<Rover5_ROS::rover_in>("RoverMSGin", 10);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_TeleOp::joyCallback, this);
	std::cout << "Finished init of Joy_Teleop" << std::endl;
}

void Joy_TeleOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//geometry_msgs::Twist twist;
	//twist.linear.x = l_scale_*joy->axes[linear_];
	//twist.angular.z = a_scale_*joy->axes[angular_];
	//vel_pub_.publish(twist);

	if((joy->axes[left_CMD])>0){
		rover_msg_in.lDirCmd = 1;
	}else{
		rover_msg_in.lDirCmd = 0;
	}

	if((joy->axes[right_CMD])>0){
		rover_msg_in.rDirCmd = 1;
	}else{
		rover_msg_in.rDirCmd = 0;
	}

	rover_msg_in.lDutyCmd = abs(100*joy->axes[left_CMD]);
	rover_msg_in.rDutyCmd = abs(100*joy->axes[right_CMD]);

	rover_pub.publish(rover_msg_in);

	/*std::cout << scratch.lDirCmd << ", " << scratch.rDirCmd << ", " << joy->axes[left_CMD]
		<< ", " << scratch.lDutyCmd << ", " << scratch.rDutyCmd << ", " << joy->axes[right_CMD] << std::endl;*/
}

Joy_TeleOp::~Joy_TeleOp() {
	std::cout << "Joy_TeleOp object terminated" << std::endl;
}


