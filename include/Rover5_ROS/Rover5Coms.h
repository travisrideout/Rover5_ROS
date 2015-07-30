/*
 * Rover5Coms.h
 *
 *  Created on: Dec 29, 2014
 *      Author: User
 */

#ifndef ROVER5COMS_H_
#define ROVER5COMS_H_

#include <string>
#include <sstream>
#include <iomanip>			// for hex
#include <stdio.h>			// for sscanf

#define PACKET_SIZE		80			// total # of characters in packet, characters because socket sends char[]
#define BLOCK_SIZE		4 			// # of characters per variable
#define PACKET_START 	"STRT"		// start string used for checking packet start
#define PACKET_END		"END0\0" 	// end string used for checking packet end
#define PORT 			12345

//if adding more data types update PACKET_SIZE
enum Data {
	START,
	CHK_SUM,
	L_DIR_CMD,
	R_DIR_CMD,
	L_DUTY_CMD,
	R_DUTY_CMD,
	L_SPEED_ACT,
	R_SPEED_ACT,
	PING_DIST,
	IMU_X_ACCEL,
	IMU_Y_ACCEL,
	IMU_Z_ACCEL,
	IMU_X_GYRO,
	IMU_Y_GYRO,
	IMU_Z_GYRO,
	L_POS_HIGH,
	L_POS_LOW,
	R_POS_HIGH,
	R_POS_LOW,
	END
};

typedef struct data_struct {
	std::string start;
	unsigned short checkSum;
	unsigned short lDirCmd;
	unsigned short rDirCmd;
	unsigned short lDutyCmd;
	unsigned short rDutyCmd;
	unsigned short lSpeedAct;
	unsigned short rSpeedAct;
	unsigned short pingDist;
	short imuXAccel;
	short imuYAccel;
	short imuZAccel;
	short imuXGyro;
	short imuYGyro;
	short imuZGyro;
	int lPos;
	int rPos;
	std::string end;
}data_struct;

// Pass message buffer (char[])and data_struct that holds the values to send
inline int PrepareSendPacket(char msg[], data_struct &data){
	std::string checkLength;
	std::stringstream ssprepare;
	ssprepare << data.start;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.checkSum;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.lDirCmd;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.rDirCmd;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.lDutyCmd;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.rDutyCmd;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.lSpeedAct;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.rSpeedAct;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.pingDist;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.imuXAccel;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.imuYAccel;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.imuZAccel;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.imuXGyro;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.imuYGyro;
	ssprepare << std::setw(4) << std::setfill('0') << std::hex << data.imuZGyro;
	ssprepare << std::setw(8) << std::setfill('0') << std::hex << data.lPos;
	ssprepare << std::setw(8) << std::setfill('0') << std::hex << data.rPos;
	ssprepare << data.end;
	ssprepare >> msg;
	return 0;
}

//pass a pointer to the received message and a pointer to a data_struct to hold the parsed values
//returns 0 on successful message parsing and -1 on error
//does not include any value calculations ie twos compliments for negative values
inline int ParseRecvPacket(char msg[], data_struct &data){
	char blockRead[BLOCK_SIZE+1];					//create char array to process blocks
	char intBlockTemp[BLOCK_SIZE*2+1];				//used for combining low/high int of position
	std::stringstream ssconvert;					//create string stream object
	short myVar = 0;								//used for signed conversion for IMU values
	int myInt = 0;									//used for signed conversion for position values
	for (int i=0; i<PACKET_SIZE; i+=BLOCK_SIZE){	//parse message in block size portions
		ssconvert.clear();							//clear string stream, have to do this for loop to work
		memset(blockRead,'0',sizeof(blockRead));	//clears blockRead
		memcpy(blockRead,&msg[i],BLOCK_SIZE);		//puts block size portion of message into block read
		blockRead[BLOCK_SIZE]= '\0';				//add termination symbol to end of block read
		if((i/BLOCK_SIZE) == START ||(i/BLOCK_SIZE) == END){			//converts the two strings
			ssconvert << blockRead;
		}else {
			ssconvert << std::hex << blockRead;		//converts blocks to hex
		}
		switch (i/BLOCK_SIZE){						//places values into data_struct members
		case START: ssconvert >> data.start;
			break;
		case CHK_SUM: ssconvert >> data.checkSum;
			break;
		case L_DIR_CMD: ssconvert >> data.lDirCmd ;
			break;
		case R_DIR_CMD: ssconvert >> data.rDirCmd;
			break;
		case L_DUTY_CMD: ssconvert >> data.lDutyCmd;
			break;
		case R_DUTY_CMD: ssconvert >> data.rDutyCmd;
			break;
		case L_SPEED_ACT: ssconvert >> data.lSpeedAct;
			break;
		case R_SPEED_ACT: ssconvert >> data.rSpeedAct;
			break;
		case PING_DIST: ssconvert >> data.pingDist;
			break;
		case IMU_X_ACCEL: sscanf(blockRead,"%hx",&myVar);
			data.imuXAccel = myVar;
			break;
		case IMU_Y_ACCEL: sscanf(blockRead,"%hx",&myVar);
			data.imuYAccel = myVar;
			break;
		case IMU_Z_ACCEL: sscanf(blockRead,"%hx",&myVar);
			data.imuZAccel = myVar;
			break;
		case IMU_X_GYRO: sscanf(blockRead,"%hx",&myVar);
			data.imuXGyro = myVar;
			break;
		case IMU_Y_GYRO: sscanf(blockRead,"%hx",&myVar);
			data.imuYGyro = myVar;
			break;
		case IMU_Z_GYRO: sscanf(blockRead,"%hx",&myVar);
			data.imuZGyro = myVar;
			break;
		case L_POS_HIGH:	memcpy(intBlockTemp,&blockRead[0],BLOCK_SIZE);
			break;
		case L_POS_LOW:
			for(int x = BLOCK_SIZE; x < BLOCK_SIZE*2; x++) {
				intBlockTemp[x] = blockRead[x-BLOCK_SIZE];
			}
			intBlockTemp[8] = '\0';
			sscanf(intBlockTemp,"%x",&myInt);
			data.lPos = myInt;
			memset(intBlockTemp,'0',sizeof(intBlockTemp));	//clears intBlockRead
			break;
		case R_POS_HIGH:	memcpy(intBlockTemp,&blockRead[0],BLOCK_SIZE);
			break;
		case R_POS_LOW:
			for(int x = BLOCK_SIZE; x < BLOCK_SIZE*2; x++) {
				intBlockTemp[x] = blockRead[x-BLOCK_SIZE];
			}
			intBlockTemp[8] = '\0';
			sscanf(intBlockTemp,"%x",&myInt);
			data.rPos = myInt;
			memset(intBlockTemp,'0',sizeof(intBlockTemp));	//clears intBlockRead
			break;
		case END: ssconvert >> data.end;
			break;
		default: std::cerr << "Packet block parsing error" << std::endl;
			return -1;
		}
	}
	return 0;
}

// Initialize values
inline void InitializeMessageData(data_struct &data){
	data.start = PACKET_START;
	data.checkSum = 0;
	data.lDirCmd = 0;
	data.rDirCmd = 0;
	data.lDutyCmd = 0;
	data.rDutyCmd = 0;
	data.lSpeedAct = 0;
	data.rSpeedAct = 0;
	data.pingDist = 0;
	data.imuXAccel = 0;
	data.imuYAccel = 0;
	data.imuZAccel = 0;
	data.imuXGyro = 0;
	data.imuYGyro = 0;
	data.imuZGyro = 0;
	data.lPos = 0;
	data.rPos = 0;
	data.end = PACKET_END;
}

#endif /* ROVER5COMS_H_ */
