/*
 * sonar.h
 *
 *  Created on: Apr 15, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_SONAR_H_
#define INCLUDE_ROVER5_ROS_SONAR_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <Rover5_ROS/rover_out.h>
#include <sensor_msgs/Range.h>

class Sonar
{
public:
	Sonar();
	virtual ~Sonar();

private:
	void PingCallback(const Rover5_ROS::rover_out::ConstPtr&);

	ros::Subscriber rover_sub;

	ros::NodeHandle nHandle;

	tf::Quaternion tf_quaternion;
	geometry_msgs::Quaternion msg_quaternion;	//quaternion for transform

	ros::Publisher range_pub;
	sensor_msgs::Range range_msg;
	geometry_msgs::TransformStamped range_tf;	//transform object
	tf::TransformBroadcaster rangeBroadcaster;

};


#endif /* INCLUDE_ROVER5_ROS_SONAR_H_ */
