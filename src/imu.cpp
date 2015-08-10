/*
 * imu.cpp
 *
 *  Created on: Apr 4, 2015
 *      Author: travis
 */

#include <Rover5_ROS/imu.h>

IMU::IMU(){
	//Publishers
	imu_pub = nHandle.advertise<sensor_msgs::Imu>("imu/data_raw", 10);

	//Subscribers
	rover_sub = nHandle.subscribe<Rover5_ROS::rover_out>("RoverMSGout", 10, &IMU::IMUCallback, this);
}

void IMU::IMUCallback(const Rover5_ROS::rover_out::ConstPtr& msg){
	imu_msg.header.stamp = msg->header.stamp;
	imu_msg.header.frame_id = "imu";

	float ax_f, ay_f, az_f;
	float gx_f, gy_f, gz_f;

	// +/-4g scale converted to m/s^2
	ax_f =((float) msg->imuXAccel) / ((SIGNED_16/4.0f) / GRAVITY);
	ay_f =((float) msg->imuYAccel) / ((SIGNED_16/4.0f) / GRAVITY);
	az_f =((float) msg->imuZAccel) / ((SIGNED_16/4.0f) / GRAVITY);

	// +/-250 degrees/s scale converted to rad/s
	gx_f=((float) msg->imuXGyro) / ((SIGNED_16/250.0f) / (M_PI/180.0));
	gy_f=((float) msg->imuYGyro) / ((SIGNED_16/250.0f) / (M_PI/180.0));
	gz_f=((float) msg->imuZGyro) / ((SIGNED_16/250.0f) / (M_PI/180.0));

	//deadband

	float nom = 0.0;
	float gyro_dev = 0.02;	//~1deg/s^2
	float accel_dev = 0.1;	//.01g's
	ax_f = Deadband(ax_f,nom,accel_dev);
	ay_f = Deadband(ay_f,nom,accel_dev);
	az_f = Deadband(az_f,9.807,accel_dev);
	gx_f = Deadband(gx_f,nom,gyro_dev);
	gy_f = Deadband(gy_f,nom,gyro_dev);
	gz_f = Deadband(gz_f,nom,gyro_dev);

	//x&y swapped and inverted due to imu orientation on robot
	imu_msg.linear_acceleration.x=ay_f;
	imu_msg.linear_acceleration.y=ax_f;
	imu_msg.linear_acceleration.z=az_f;

	imu_msg.angular_velocity.x=gy_f;
	imu_msg.angular_velocity.y=-gx_f;
	imu_msg.angular_velocity.z=-gz_f;

	imu_pub.publish(imu_msg);
}

float IMU::Deadband(float value, float nom, float dev){
	if(value > (nom-dev) && value < (nom+dev)){
		value = nom;
	}
	return value;
}
IMU::~IMU(){
	std::cerr << "IMU object terminated" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_imu");

	IMU imu;

	ros::Rate loop_rate(FREQUENCY);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}


