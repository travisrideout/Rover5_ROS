/*
 * sonar.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: travis
 */

#include <Rover5_ROS/sonar.h>

Sonar::Sonar(){
	range_pub = nHandle.advertise<sensor_msgs::Range>("range", 10);
	rover_sub = nHandle.subscribe<Rover5_ROS::rover_out>("RoverMSGout", 10, &Sonar::PingCallback, this);
}

void Sonar::PingCallback(const Rover5_ROS::rover_out::ConstPtr& msg){
	range_msg.header.frame_id = "range";
	range_msg.header.stamp = msg->header.stamp;
	range_msg.radiation_type = 0;
	range_msg.field_of_view = 0.1;
	range_msg.min_range = 0.05;
	range_msg.max_range = 15;
	range_msg.range = msg->pingDist/100;	//convert from cm to meters

	range_pub.publish(range_msg);

	range_tf.header.stamp = msg->header.stamp;
	range_tf.header.frame_id = "range";
	range_tf.child_frame_id = "odom";
	range_tf.transform.translation.x = 0;
	range_tf.transform.translation.y = 0;
	range_tf.transform.translation.z = 0;
	tf_quaternion = tf::createQuaternionFromYaw(0);
	tf::quaternionTFToMsg(tf_quaternion, msg_quaternion);
	range_tf.transform.rotation = msg_quaternion;

	//send the transform
	rangeBroadcaster.sendTransform(range_tf);
}

Sonar::~Sonar(){
	std::cout << "Sonar object destroyed" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_sonar");
	std::cout << "ros sonar node started" <<std::endl;

	Sonar sonar;

	ros::Rate loop_rate(30);
	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}
}
