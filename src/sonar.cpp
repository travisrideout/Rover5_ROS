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
	range_msg.min_range = 0.01;
	range_msg.max_range = 3.0;
	range_msg.range = msg->pingDist/100.0;	//convert from cm to meters

	//Set values outside max/min setting to those values, used to clear sensor cone in costmap
	//error readings, greater than 30m ignored
	if(range_msg.range>range_msg.max_range && range_msg.range<30.0){
		range_msg.range = range_msg.max_range;
	}else if(range_msg.range<range_msg.min_range){
		range_msg.range = range_msg.min_range;
	}

	range_pub.publish(range_msg);

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
