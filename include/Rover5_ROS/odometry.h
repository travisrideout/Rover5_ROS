/*
 * odometry.h
 *
 *  Created on: Mar 20, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_ODOMETRY_H_
#define INCLUDE_ROVER5_ROS_ODOMETRY_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <limits.h>
#include <Rover5_ROS/rover_out.h>
#include <Rover5_ROS/rover_in.h>
#include <Rover5_ROS/joy_teleop.h>

class Odometry{
public:
	Odometry();
	void Diff_To_Twist();
	void Update_Odom();
	virtual ~Odometry();
private:
	void EncCallback(const Rover5_ROS::rover_out::ConstPtr&);
	void Twist_To_Diff(const geometry_msgs::TwistWithCovariance::ConstPtr&);

	int rolloverMax, rolloverMin;				//encoder count roll over limits
	int ratio;									//encoder ticks per meter
	float width;								//robot width in meters
	float x,y,prev_x,prev_y;					//position and previous values to compute velocities
	float th, prev_th;							// heading angle and previous
	ros::Time now, then; 						//times to compute velocities
	float elapsed;
	int l_enc, r_enc,prev_l_enc,prev_r_enc;		//encoder tick values
	float dx,dth;								//velocities

	tf::Quaternion tf_quaternion;
	geometry_msgs::Quaternion msg_quaternion;	//quaternion for transform
	geometry_msgs::TransformStamped odom_tf;	//transform object
	Rover5_ROS::rover_in rover_msg_in;			//rover input object

	ros::Publisher odom_pub;
	ros::Publisher rover_pub;
	ros::Subscriber rover_sub;
	tf::TransformBroadcaster odomBroadcaster;
	ros::Subscriber twist_sub;

	ros::NodeHandle nHandle;

	nav_msgs::Odometry odom_msg;
};

#endif /* INCLUDE_ROVER5_ROS_ODOMETRY_H_ */
