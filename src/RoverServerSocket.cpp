
#include <Rover5_ROS/RoverServerSocket.h>

RoverServerSocket::RoverServerSocket() {
	InitializeMessageData(msgSendData);
	rover_pub = n.advertise<Rover5_ROS::rover>("RoverMSG", 10);
}

void* RoverServerSocket::ServerThreadStarter(void* context){
	return((RoverServerSocket *)context)->StartServer();
}

void* RoverServerSocket::StartServer(){

	std::cout << "Starting Server" << std::endl; 

    int socket_desc , client_sock , c , read_size;
    struct sockaddr_in server , client;
     
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
     
    while(1){
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

    	ros::Rate loop_rate(60);

    	//Receive a message from client
    	while( (read_size = recv(client_sock, msgRecv, PACKET_SIZE, 0)) > 0 )
    	{
    		ParseRecvPacket(msgRecv, msgRecvData);
    		UseMessageData();
    		loop_rate.sleep();
    		pthread_mutex_lock(&lock);
    		PackSendData();
    		PrepareSendPacket(msgSend, msgSendData);
    		send(client_sock, msgSend, PACKET_SIZE, 0);
    		//std::cout << msgSend << std::endl;
    		pthread_mutex_unlock(&lock);
    	}

    	if(read_size == 0)
    	{
    		std::cerr << "Client disconnected" << std::endl;
    	}
    	else if(read_size == -1)
    	{
    		std::cerr << "recv failed" << std::endl;
    	}
    }
     
    return 0;
}

int RoverServerSocket::UseMessageData(){
	rover_msg.header.stamp = ros::Time::now();
	rover_msg.header.frame_id = "/world";
	rover_msg.pingDist = SF.AveragingFilter(msgRecvData.pingDist);
	rover_msg.lPos = msgRecvData.lPos;
	rover_msg.rPos = msgRecvData.rPos;
	rover_msg.imuXAccel = msgRecvData.imuXAccel;
	rover_msg.imuYAccel = msgRecvData.imuYAccel;
	rover_msg.imuZAccel = msgRecvData.imuZAccel;

	rover_pub.publish(rover_msg);

	std::cout << "\tPing = " << msgRecvData.pingDist
			<< "\tL_POS = " << msgRecvData.lPos
			<< "\tR_POS = " << msgRecvData.rPos
			<< "\tX accel = " <<  (float)msgRecvData.imuXAccel/16384
			<< "\tY accel = " << (float)msgRecvData.imuYAccel/16384
			<< "\tZ accel = " << (float)msgRecvData.imuZAccel/16384 << std::endl;

	return 0;
}

int RoverServerSocket::PackSendData(){
	msgSendData.lDutyCmd = rover_msg.lDutyCmd;
	msgSendData.rDutyCmd = rover_msg.rDutyCmd;
	msgSendData.lDirCmd = rover_msg.lDirCmd;
	msgSendData.rDirCmd = rover_msg.rDirCmd;

	return 0;
}

int RoverServerSocket::GetMessageVars(data_struct *new_vars){
	*new_vars = msgRecvData;
	return 1;
}

int RoverServerSocket::SetMessageVars(data_struct *new_vars){
	msgSendData = *new_vars;
	return 1;
}

RoverServerSocket::~RoverServerSocket() {
	std::cout << "RoverServer object terminated" << std::endl;
}


