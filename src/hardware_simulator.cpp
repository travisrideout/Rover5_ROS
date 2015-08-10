/*
 * hardware_simulator.cpp
 *
 *  Created on: Jul 18, 2015
 *      Author: travis
 */

#include <Rover5_ROS/hardware_simulator.h>

HWSim::HWSim():left_enc(0),right_enc(0),
	XAccel(0),YAccel(0),ZAccel(0),XGyro(0),YGyro(0),ZGyro(0), ping(0),
	lp_0(0), lp_1(0), rp_0(0), rp_1(0), lv_des(0), rv_des(0), lv_0(0),
	lv_1(0), rv_0(0), rv_1(0), elapsed(0){

	now = ros::Time::now();
	then = ros::Time::now();

	//Subscribers
	twist_sub = nh_.subscribe("rover_cmd_vel", 10, &HWSim::TwistCallback, this);

	//Publishers
	rover_pub = nh_.advertise<Rover5_ROS::rover_out>("RoverMSGout", 10);
}

void HWSim::TwistCallback(const geometry_msgs::TwistWithCovariance& twist){
	lv_des = (twist.twist.linear.x + ((WIDTH*twist.twist.angular.z)/2)) * max_vel;
	rv_des = (twist.twist.linear.x - ((WIDTH*twist.twist.angular.z)/2)) * max_vel;
}

// check desired velocity against previous velocity, if different accelerate towards desired
// actual velocity = previous velocity + acceleration/elapsed time until actual velocity = desired velocity
// current position = previous position + velocity/elapsed time
void HWSim::OdomSim(){
	now = ros::Time::now();
	elapsed = (now-then).toSec();
	if(lv_des>(lv_1+deadband)){
		if(lv_1>=max_vel){
			lv_0 = max_vel;
		}else{
			lv_0 = lv_1 + accel_rate*elapsed;
		}
	}else if(lv_des<(lv_1-deadband)){
		if(lv_1<=-max_vel){
			lv_0 = -max_vel;
		}else{
			lv_0 = lv_1 - accel_rate*elapsed;
		}
	}else{
		lv_0 = lv_1;
	}
	lp_0 = lp_1 + lv_0*elapsed;

	if(rv_des>(rv_1+deadband)){
		if(rv_1>=max_vel){
			rv_0 = max_vel;
		}else{
			rv_0 = rv_1 + accel_rate*elapsed;
		}
	}else if(rv_des<(rv_1-deadband)){
		if(rv_1<=-max_vel){
			rv_0 = -max_vel;
		}else{
			rv_0 = rv_1 - accel_rate*elapsed;
		}
	}else{
		rv_0 = rv_1;
	}
	rp_0 = rp_1 + rv_0*elapsed;

	//set encoder counts
	left_enc = (int)(lp_0 * ENC_RATIO);
	right_enc = (int)(rp_0 * ENC_RATIO);
}

// x acceleration = average accel of tracks = change in vel/elapsed time
// yaw rate = delta angle / elapsed time
void HWSim::IMUSim(){
	XAccel = 0;
	YAccel = (((lv_0 - lv_1)+(rv_0 - rv_1))/(2*elapsed))*accel_scalar;
	ZAccel = 1 * accel_scalar;	// 1g, gravity
	XGyro = 0;
	YGyro = 0;
	double angle = asin(((lp_0-lp_1)-(rp_0-rp_1))/WIDTH)*(180/M_PI);	//converted to degrees
	ZGyro = (angle/elapsed)*gyro_scalar;
}

void HWSim::RangeSim(){
	ping = 0;
}

//push back variables
void HWSim::PushVars(){
	lv_1 = lv_0;
	rv_1 = rv_0;
	lp_1 = lp_0;
	rp_1 = rp_0;
	then = now;
}

void HWSim::RoverMSG(){
	OdomSim();
	IMUSim();
	RangeSim();

	rover_msg_out.header.stamp = now;
	rover_msg_out.header.frame_id = "/world";
	rover_msg_out.pingDist = ping;
	rover_msg_out.lPos = left_enc;
	rover_msg_out.rPos = right_enc;
	rover_msg_out.imuXAccel = XAccel;
	rover_msg_out.imuYAccel = YAccel;
	rover_msg_out.imuZAccel = ZAccel;
	rover_msg_out.imuXGyro = XGyro;
	rover_msg_out.imuYGyro = YGyro;
	rover_msg_out.imuZGyro = ZGyro;

	rover_pub.publish(rover_msg_out);

	PushVars();
}

HWSim::~HWSim(){
	std::cout << "Simulator object terminated" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_sim");
	HWSim sim;

	std::cout << "Hardware Simulator node started" << std::endl;

	ros::Rate loop_rate(FREQUENCY);

	while(ros::ok()){
		sim.RoverMSG();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
