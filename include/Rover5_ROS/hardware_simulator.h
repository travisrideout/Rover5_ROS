/*
 * hardware_simulator.h
 *
 *  Created on: Jul 18, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_HARDWARE_SIMULATOR_H_
#define INCLUDE_ROVER5_ROS_HARDWARE_SIMULATOR_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <Rover5_ROS/rover_out.h>
#include <math.h>

#define frequency		30.0		// Hz
#define ping_scalar 	100			// cm's
#define accel_scalar	8192		// +/-4g
#define gyro_scalar		131.072		// +/-250deg/s
#define accel_rate		0.5			// m/s^2
#define max_vel			0.3			// m/s
#define enc_ratio		1768		// encoder counts per meter
#define width			0.1905		// robot width in meters
#define deadband		0.001		// joy deadband, m/s

class HWSim{
public:
	HWSim();
	~HWSim();

	void RoverMSG();

private:
	void TwistCallback(const geometry_msgs::TwistWithCovariance& twist);
	void OdomSim();
	void IMUSim();
	void RangeSim();
	void PushVars();

	ros::NodeHandle nh_;

	ros::Subscriber twist_sub;
	ros::Publisher rover_pub;

	Rover5_ROS::rover_out rover_msg_out;

	double 	left_enc, right_enc;
	int 	XAccel,YAccel,ZAccel,XGyro,YGyro,ZGyro;
	int 	ping;

	//calculation variables
	double 	lp_0, lp_1, rp_0, rp_1;						// left/right positions
	float	lv_des, rv_des, lv_0, lv_1, rv_0, rv_1;		// left/right velocities
	double 	elapsed;
	ros::Time now, then;

};


#endif /* INCLUDE_ROVER5_ROS_HARDWARE_SIMULATOR_H_ */
