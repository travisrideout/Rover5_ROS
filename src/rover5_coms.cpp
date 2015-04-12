#include <Rover5_ROS/rover5_coms.h>

Rover5_Coms::Rover5_Coms():socket_desc(0),client_sock(0),c(0),read_size(0){
	InitializeMessageData(msgSendData);
	rover_pub = n.advertise<Rover5_ROS::rover_out>("RoverMSGout", 10);
	rover_sub = n.subscribe<Rover5_ROS::rover_in>("RoverMSGin", 10, &Rover5_Coms::PackSendData, this);
}

void Rover5_Coms::StartServer(){
	std::cout << "Starting Server" << std::endl;

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

int Rover5_Coms::Listen(){
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
		return -1;
	}
	std::cout << "Connection accepted" << std::endl;
	return 0;
}

int Rover5_Coms::Transmit(){
	//Receive a message from client
	if((read_size = recv(client_sock, msgRecv, PACKET_SIZE, 0)) > 0)
	{
		ParseRecvPacket(msgRecv, msgRecvData);
		UseMessageData();
		PrepareSendPacket(msgSend, msgSendData);
		send(client_sock, msgSend, PACKET_SIZE, 0);
	}else {
		if(read_size == 0){
			std::cerr << "Client disconnected" << std::endl;
		}else if(read_size == -1){
			std::cerr << "recv failed" << std::endl;
		}
		rover_msg_out.header.stamp = ros::Time::now();
		rover_msg_out.header.frame_id = "/world";
		rover_msg_out.lPos = 0;
		rover_msg_out.rPos = 0;

		rover_pub.publish(rover_msg_out);

		Listen();
	}
	return 0;
}

int Rover5_Coms::UseMessageData(){
	rover_msg_out.header.stamp = ros::Time::now();
	rover_msg_out.header.frame_id = "/world";
	rover_msg_out.pingDist = msgRecvData.pingDist;
	rover_msg_out.lPos = msgRecvData.lPos;
	rover_msg_out.rPos = msgRecvData.rPos;
	rover_msg_out.imuXAccel = msgRecvData.imuXAccel;
	rover_msg_out.imuYAccel = msgRecvData.imuYAccel;
	rover_msg_out.imuZAccel = msgRecvData.imuZAccel;
	rover_msg_out.imuXGyro = msgRecvData.imuXGyro;
	rover_msg_out.imuYGyro = msgRecvData.imuYGyro;
	rover_msg_out.imuZGyro = msgRecvData.imuZGyro;

	rover_pub.publish(rover_msg_out);

	/*std::cout
			<< "\tX Gyro = " <<  (float)msgRecvData.imuXGyro/131.072
			<< "\tY Gyro = " << (float)msgRecvData.imuYGyro/131.072
			<< "\tZ Gyro = " << (float)msgRecvData.imuZGyro/131.072 << std::endl;*/
	/*<< "\tX Accel = " <<  (float)msgRecvData.imuXAccel/16384
		<< "\tY Accel = " << (float)msgRecvData.imuYAccel/16384
		<< "\tZ Accel = " << (float)msgRecvData.imuZAccel/16384
	<< "\tPing = " << rover_msg_out.pingDist
	<< "\tL_POS = " << msgRecvData.lPos
	<< "\tR_POS = " << msgRecvData.rPos*/

	return 0;
}

void Rover5_Coms::PackSendData(const Rover5_ROS::rover_in::ConstPtr& msg){
	msgSendData.lDutyCmd = msg->lDutyCmd;
	msgSendData.rDutyCmd = msg->rDutyCmd;
	msgSendData.lDirCmd = msg->lDirCmd;
	msgSendData.rDirCmd = msg->rDirCmd;

	return;
}

Rover5_Coms::~Rover5_Coms(){
	std::cerr << "Rover5_Coms deconstructed!" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_coms");
	std::cout << "ros coms node started" <<std::endl;

	Rover5_Coms coms;

	coms.StartServer();
	coms.Listen();

	ros::Rate loop_rate(30);
	while(ros::ok()){
		coms.Transmit();
		loop_rate.sleep();
		ros::spinOnce();
	}
}
