
#ifndef ROVERSERVERSOCKET_H_
#define ROVERSERVERSOCKET_H_

#include <ros/ros.h>
#include <iostream>
#include <Rover5_ROS/Rover5Coms.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <pthread.h>
#include "Rover5_ROS/rover.h"
#include <Rover5_ROS/sonar_filter.h>

extern pthread_mutex_t lock;
extern Rover5_ROS::rover rover_msg;

class RoverServerSocket {
public:
	RoverServerSocket();

	void* StartServer();
	static void* ServerThreadStarter(void*);
	int UseMessageData();
	int PackSendData();

	//Message modifiers
	int GetMessageVars(data_struct*);
	int SetMessageVars(data_struct*);

	virtual ~RoverServerSocket();

private:
	ros::NodeHandle n;
	ros::Publisher rover_pub;
	Sonar_Filter SF;
	data_struct msgSendData;
	data_struct msgRecvData;
	char msgSend[PACKET_SIZE];
	char msgRecv[PACKET_SIZE];
};

#endif /* ROVERSERVERSOCKET_H_ */
