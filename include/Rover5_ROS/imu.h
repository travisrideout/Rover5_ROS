/*
 * imu.h
 *
 *  Created on: Apr 4, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_IMU_H_
#define INCLUDE_ROVER5_ROS_IMU_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Rover5_ROS/rover_out.h>

class IMU{
public:
	IMU();
	virtual ~IMU();
private:
	void IMUCallback(const Rover5_ROS::rover_out::ConstPtr&);
	float Deadband(float value, float min, float max);

	ros::Subscriber rover_sub;
	ros::Publisher imu_pub;

	sensor_msgs::Imu imu_msg;

	ros::NodeHandle nHandle;

};

#endif /* INCLUDE_ROVER5_ROS_IMU_H_ */
