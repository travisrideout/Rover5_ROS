#include <ros/ros.h>
#include <iostream>
#include <Rover5_ROS/Rover5Coms.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "Rover5_ROS/rover_out.h"
#include "Rover5_ROS/rover_in.h"

void StartServer();
int Listen();
int Transmit();
int UseMessageData();
void PackSendData(const Rover5_ROS::rover_in::ConstPtr&);

ros::Publisher rover_pub;
ros::Subscriber rover_sub;

data_struct msgSendData;
data_struct msgRecvData;
char msgSend[PACKET_SIZE];
char msgRecv[PACKET_SIZE];
Rover5_ROS::rover_in rover_msg_in;
Rover5_ROS::rover_out rover_msg_out;
int socket_desc , client_sock , c , read_size;
struct sockaddr_in server , client;

void StartServer(){

	std::cout << "Starting Server" << std::endl;

    //int socket_desc , client_sock , c , read_size;
    //struct sockaddr_in server , client;

    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
        std::cerr << "Could not create socket" << std::endl;
    }
    std::cout<< "Socket created" << std::endl;

    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( 12345 );

    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
        //print the error message
        std::cerr << "bind failed. Error" << std::endl;
        //return 1;
    }
    std::cout << "bind done" << std::endl;

    return;
}

int Listen(){
	//Listen
	listen(socket_desc , 3);

	//Accept and incoming connection
	std::cout<< "Waiting for incoming connections..." << std::endl;
	c = sizeof(struct sockaddr_in);

	//accept connection from an incoming client
	client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
	if (client_sock < 0)
	{
		std::cerr << "accept failed" << std::endl;
		//return 1;
	}
	std::cout << "Connection accepted" << std::endl;
	return 0;
}

int Transmit(){
	//Receive a message from client
	if((read_size = recv(client_sock, msgRecv, PACKET_SIZE, 0)) > 0)
	{
		ParseRecvPacket(msgRecv, msgRecvData);
		UseMessageData();
		//PackSendData();
		//ros::spinOnce();
		PrepareSendPacket(msgSend, msgSendData);
		send(client_sock, msgSend, PACKET_SIZE, 0);
	}else if(read_size == 0)
	{
		std::cerr << "Client disconnected" << std::endl;
		Listen();
	}
	else if(read_size == -1)
	{
		std::cerr << "recv failed" << std::endl;
		Listen();
	}
	return 0;
}

int UseMessageData(){
	//rover_msg1.stamp = ros::Time::now();
	rover_msg_out.header.frame_id = "/world";
	rover_msg_out.pingDist = msgRecvData.pingDist;
	rover_msg_out.lPos = msgRecvData.lPos;
	rover_msg_out.rPos = msgRecvData.rPos;
	rover_msg_out.imuXAccel = msgRecvData.imuXAccel;
	rover_msg_out.imuYAccel = msgRecvData.imuYAccel;
	rover_msg_out.imuZAccel = msgRecvData.imuZAccel;

	rover_pub.publish(rover_msg_out);

	std::cout << "\tPing = " << rover_msg_out.pingDist
			<< "\tL_POS = " << msgRecvData.lPos
			<< "\tR_POS = " << msgRecvData.rPos
			<< "\tX accel = " <<  (float)msgRecvData.imuXAccel/16384
			<< "\tY accel = " << (float)msgRecvData.imuYAccel/16384
			<< "\tZ accel = " << (float)msgRecvData.imuZAccel/16384 << std::endl;

	return 0;
}

void PackSendData(const Rover5_ROS::rover_in::ConstPtr& msg){
	msgSendData.lDutyCmd = msg->lDutyCmd;
	msgSendData.rDutyCmd = msg->rDutyCmd;
	msgSendData.lDirCmd = msg->lDirCmd;
	msgSendData.rDirCmd = msg->rDirCmd;

	return;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_coms");
	std::cout << "ros coms node started" <<std::endl;

	ros::NodeHandle n;

	InitializeMessageData(msgSendData);
	rover_pub = n.advertise<Rover5_ROS::rover_out>("RoverMSGout", 10);
	rover_sub = n.subscribe<Rover5_ROS::rover_in>("RoverMSGin", 10, &PackSendData);

	StartServer();
	Listen();

	ros::Rate loop_rate(60);
	while(ros::ok()){
		Transmit();
		loop_rate.sleep();
		ros::spinOnce();
	}
}
