#include <ros/ros.h>
//#include <Rover5_ROS/RoverServerSocket.h>
#include <Rover5_ROS/Rover5Coms.h>
#include <Rover5_ROS/joy_teleop.h>
#include "Rover5_ROS/rover.h"

//#include <pthread.h>		// for threading

//pthread_t serverThread;		//thread object
//pthread_mutex_t lock;		//thread lock object

//Rover5_ROS::rover rover_msg;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rover5_ros");
	//ros::init(argc, argv, "rover_msg");

	//ros::NodeHandle nHandle;

	Joy_TeleOp joy_teleop;

	//RoverServerSocket tcp;

	//pthread_mutex_init(&lock, NULL);
	//pthread_create(&serverThread, NULL, &RoverServerSocket::ServerThreadStarter, &tcp);

	ros::spin();
}
