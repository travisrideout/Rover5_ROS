/*
 * rover5_coms.h
 *
 *  Created on: Apr 8, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_ROVER5_COMS_H_
#define INCLUDE_ROVER5_ROS_ROVER5_COMS_H_

#include <ros/ros.h>
#include <iostream>
#include <Rover5_ROS/Rover5Coms.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <Rover5_ROS/rover_out.h>
#include <Rover5_ROS/rover_in.h>
#include <Rover5_ROS/rover5_param.h>

class Rover5_Coms{
public:
	Rover5_Coms();
	void StartServer();
	int Listen();
	int Transmit();
	virtual ~Rover5_Coms();
private:
	int UseMessageData();
	void PackSendData(const Rover5_ROS::rover_in::ConstPtr&);
	void TwistCallback(const geometry_msgs::TwistWithCovariance& twist);

	//Publishers
	ros::Publisher rover_pub_out;
	ros::Publisher rover_pub_in;

	//Subscribers
	ros::Subscriber rover_sub;
	ros::Subscriber twist_sub;

	ros::NodeHandle n;

	data_struct msgSendData;
	data_struct msgRecvData;
	char msgSend[PACKET_SIZE];
	char msgRecv[PACKET_SIZE];
	Rover5_ROS::rover_in rover_msg_in;
	Rover5_ROS::rover_out rover_msg_out;
	int socket_desc , client_sock , c , read_size;
	struct sockaddr_in server , client;
};


#endif /* INCLUDE_ROVER5_ROS_ROVER5_COMS_H_ */
