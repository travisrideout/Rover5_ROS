/*
 * odometry.h
 *
 *  Created on: Mar 20, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_ODOMETRY_H_
#define INCLUDE_ROVER5_ROS_ODOMETRY_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <limits.h>
#include <Rover5_ROS/rover_out.h>
#include <sensor_msgs/Range.h>
#include <Rover5_ROS/rover5_param.h>

class Odometry{
public:
	Odometry();
	void Update_Odom();
	virtual ~Odometry();

private:
	void EncCallback(const Rover5_ROS::rover_out::ConstPtr&);

	int rolloverMax, rolloverMin;				//encoder count roll over limits
	double x,y,prev_x,prev_y;					//position and previous values to compute velocities
	double th, prev_th;							// heading angle and previous
	ros::Time now, then; 						//times to compute velocities
	double elapsed;
	int l_enc, r_enc,prev_l_enc,prev_r_enc;		//encoder tick values
	double dx,dth;								//velocities

	tf::Quaternion tf_quaternion;
	geometry_msgs::Quaternion msg_quaternion;	//quaternion for transform

	//Publishers
	ros::Publisher odom_pub;

	//Subscribers
	ros::Subscriber rover_sub;

	ros::NodeHandle nHandle;

	nav_msgs::Odometry odom_msg;
};

#endif /* INCLUDE_ROVER5_ROS_ODOMETRY_H_ */
