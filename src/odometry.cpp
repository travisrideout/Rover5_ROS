/*
 * odometry.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: travis
 */

#include <Rover5_ROS/odometry.h>

Odometry::Odometry():
	x(0.0),y(0.0),prev_x(0.0),prev_y(0.0),th(0.0),prev_th(0.0),now(0.0),then(0.0), elapsed(0.0),
	l_enc(0), r_enc(0), prev_l_enc(0), prev_r_enc(0), dx(0.0),dth(0.0){

	rolloverMax = 0.95*std::numeric_limits<int>::max();
	rolloverMin = 0.95*std::numeric_limits<int>::min();

	//Publishers
	odom_pub = nHandle.advertise<nav_msgs::Odometry>("odom", 10);

	//Subscribers
	rover_sub = nHandle.subscribe<Rover5_ROS::rover_out>("RoverMSGout", 10, &Odometry::EncCallback, this);
}

void Odometry::EncCallback(const Rover5_ROS::rover_out::ConstPtr& msg){
	//TODO: create roll over safeties for encoder counts
	l_enc = msg->lPos;
	r_enc = msg->rPos;
	now = msg->header.stamp;
}

void Odometry::Update_Odom(){
	if(now.nsec != 0 && then.nsec != 0 && now.nsec>then.nsec){
		elapsed = (now-then).toSec();

		//calculate odometry
		double d_left = (double(l_enc) - prev_l_enc)/ENC_RATIO;		//distance traveled by each track
		double d_right = (double(r_enc) - prev_r_enc)/ENC_RATIO;

		//distance traveled is the average of the two wheels
		double d = ( d_left + d_right ) / 2;
		//this approximation works (in radians) for small angles
		double th_temp = ( d_right - d_left ) / (2*WIDTH);			//divide by 2 to reduce errors in theta

		//calculate velocities
		dx = d / elapsed;
		dth = th_temp / elapsed;

		if (d != 0){
			//calculate distance traveled in x and y
			double x_temp = cos( th_temp ) * d;
			double y_temp = -sin( th_temp ) * d;
			//calculate the final position of the robot
			x += ( cos( th ) * x_temp - sin( th ) * y_temp );
			y += ( sin( th ) * x_temp + cos( th ) * y_temp );
		}

		//TODO: Should theta be limited to 1 revolution and rolled over?
		if(th_temp != 0){
			th += th_temp;
		}

		//pack the odometry
		odom_msg.header.stamp = now;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_footprint";
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0;
		tf_quaternion = tf::createQuaternionFromYaw(th);
		tf::quaternionTFToMsg(tf_quaternion, msg_quaternion);
		odom_msg.pose.pose.orientation = msg_quaternion;
		odom_msg.twist.twist.linear.x = dx;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.angular.z = dth;

		//Covariance values. Total guess
		odom_msg.pose.covariance[0]  = 0.001;
		odom_msg.pose.covariance[7]  = 0.001;
		odom_msg.pose.covariance[14] = 100000;
		odom_msg.pose.covariance[21] = 100000;
		odom_msg.pose.covariance[28] = 100000;
		odom_msg.pose.covariance[35] = 100;

		//send the odometry
		odom_pub.publish(odom_msg);
	}

	//push current values to buffers
	then = now;
	prev_l_enc = l_enc;
	prev_r_enc = r_enc;
}

Odometry::~Odometry(){
	std::cout << "Odometry object destroyed" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_odom");
	Odometry odom;

	ros::Rate loop_rate(FREQUENCY);

	while(ros::ok()){
		odom.Update_Odom();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

