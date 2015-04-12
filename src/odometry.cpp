/*
 * odometry.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: travis
 */

#include <Rover5_ROS/odometry.h>

Odometry::Odometry():
	ratio(1768),width(0.1905),
	x(0.0),y(0.0),prev_x(0.0),prev_y(0.0),th(0.0),prev_th(0.0),now(0.0),then(0.0), elapsed(0.0),
	l_enc(0), r_enc(0), prev_l_enc(0), prev_r_enc(0),
	dx(0.0),dth(0.0){
	rolloverMax = 0.95*std::numeric_limits<int>::max();
	rolloverMin = 0.95*std::numeric_limits<int>::min();

	//Publishers
	odom_pub = nHandle.advertise<nav_msgs::Odometry>("odom", 10);
	rover_pub = nHandle.advertise<Rover5_ROS::rover_in>("RoverMSGin", 10);

	range_pub = nHandle.advertise<sensor_msgs::Range>("range", 10);

	//Subscribers
	rover_sub = nHandle.subscribe<Rover5_ROS::rover_out>("RoverMSGout", 10, &Odometry::EncCallback, this);
	twist_sub = nHandle.subscribe<geometry_msgs::TwistWithCovariance>("rover_cmd_vel", 10, &Odometry::Twist_To_Diff, this);
}

void Odometry::EncCallback(const Rover5_ROS::rover_out::ConstPtr& msg){
	//TODO: create roll over safeties for encoder counts
	l_enc = msg->lPos;
	r_enc = msg->rPos;
	now = msg->header.stamp;

	range_msg.header.frame_id = "range";
	range_msg.header.stamp = msg->header.stamp;
	range_msg.radiation_type = 0;
	range_msg.field_of_view = 0.1;
	range_msg.min_range = 0.05;
	range_msg.max_range = 15;
	range_msg.range = msg->pingDist/100;

	range_pub.publish(range_msg);
}

void Odometry::Twist_To_Diff(const geometry_msgs::TwistWithCovariance::ConstPtr& twist){
	float r_duty_temp = twist->twist.linear.x + ((width*twist->twist.angular.z)/2);
	float l_duty_temp = twist->twist.linear.x - ((width*twist->twist.angular.z)/2);

	if(l_duty_temp<0){
		rover_msg_in.lDirCmd = 0;
	}else if(l_duty_temp>0){
		rover_msg_in.lDirCmd = 1;
	}
	if(r_duty_temp<0){
		rover_msg_in.rDirCmd = 0;
	}else if((r_duty_temp>0)){
		rover_msg_in.rDirCmd = 1;
	}

	rover_msg_in.lDutyCmd = abs(100*l_duty_temp);
	rover_msg_in.rDutyCmd = abs(100*r_duty_temp);

	rover_pub.publish(rover_msg_in);
}

void Odometry::Update_Odom(){
	if(now.nsec != 0 && then.nsec != 0 && now.nsec>then.nsec){
		elapsed = (now-then).toSec();

		//calculate odometry
		double d_left = (double(l_enc) - prev_l_enc)/ratio;		//distance traveled by each track
		double d_right = (double(r_enc) - prev_r_enc)/ratio;

		//distance traveled is the average of the two wheels
		double d = ( d_left + d_right ) / 2;
		//this approximation works (in radians) for small angles
		double th_temp = ( d_right - d_left ) / (2*width);			//divide by to to reduce errors in theta

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

		//pack up transform
		tf_quaternion = tf::createQuaternionFromYaw(th);
		tf::quaternionTFToMsg(tf_quaternion, msg_quaternion);
		odom_tf.header.stamp = now;
		odom_tf.header.frame_id = "odom";
		odom_tf.child_frame_id = "base_link";
		odom_tf.transform.translation.x = x;
		odom_tf.transform.translation.y = y;
		odom_tf.transform.translation.z = 0.0;
		odom_tf.transform.rotation = msg_quaternion;

		//send the transform
		odomBroadcaster.sendTransform(odom_tf);

		range_tf.header.stamp = now;
		range_tf.header.frame_id = "range";
		range_tf.child_frame_id = "odom";
		range_tf.transform.translation.x = 0;
		range_tf.transform.translation.y = 0;
		range_tf.transform.translation.z = 0;

		//send the transform
		rangeBroadcaster.sendTransform(range_tf);

		//pack the odometry
		odom_msg.header.stamp = now;
		odom_msg.header.frame_id = "odom";
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0;
		odom_msg.pose.pose.orientation = msg_quaternion;
		odom_msg.child_frame_id = "base_link";
		odom_msg.twist.twist.linear.x = dx;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.angular.z = dth;

		//Covariance values. Total guess
		odom_msg.pose.covariance[0] = 0.01;
		odom_msg.pose.covariance[7]  = 0.01;
		odom_msg.pose.covariance[14] = 99999;
		odom_msg.pose.covariance[21] = 99999;
		odom_msg.pose.covariance[28] = 99999;
		odom_msg.pose.covariance[35] = 0.01;

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
	Joy_TeleOp joy_teleop;
	Odometry odom;

	ros::Rate loop_rate(30);

	while(ros::ok()){
		odom.Update_Odom();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

