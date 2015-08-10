/*
 * map.h
 *
 *  Created on: Jun 23, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_MAP_H_
#define INCLUDE_ROVER5_ROS_MAP_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <Rover5_ROS/rover5_param.h>

class Map{
public:
	Map();
	void Update_Map();
	virtual ~Map();

private:
	tf::Quaternion tf_quaternion;
	geometry_msgs::Quaternion msg_quaternion;	//quaternion for transform
	geometry_msgs::TransformStamped map_tf;		//transform object

	tf::TransformBroadcaster mapBroadcaster;

};

#endif /* INCLUDE_ROVER5_ROS_ODOMETRY_H_ */
