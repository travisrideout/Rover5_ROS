/*
 * imu.cpp
 *
 *  Created on: Apr 4, 2015
 *      Author: travis
 */

#include <Rover5_ROS/imu.h>

IMU::IMU(){
	//Publishers
	imu_pub = nHandle.advertise<sensor_msgs::Imu>("imu_data", 10);

	//Subscribers
	rover_sub = nHandle.subscribe<Rover5_ROS::rover_out>("RoverMSGout", 10, &IMU::IMUCallback, this);
}

void IMU::IMUCallback(const Rover5_ROS::rover_out::ConstPtr& msg){
	imu_msg.header.stamp = msg->header.stamp;
	imu_msg.header.frame_id = "imu";

	//Set covariance values
	imu_msg.linear_acceleration_covariance[0] = .1;
	imu_msg.linear_acceleration_covariance[4] = .1;
	imu_msg.linear_acceleration_covariance[8] = .1;

	imu_msg.angular_velocity_covariance[0] = 5;
	imu_msg.angular_velocity_covariance[4] = 5;
	imu_msg.angular_velocity_covariance[8] = 5;

	imu_msg.orientation_covariance[0] = 1;
	imu_msg.orientation_covariance[4] = 1;
	imu_msg.orientation_covariance[8] = 1;

	float ax_f, ay_f, az_f;
	float gx_f, gy_f, gz_f;

	ax_f =((float) msg->imuXAccel) / (16384 / 9.807); // 2g scale in m/s^2
	ay_f =((float) msg->imuYAccel) / (16384 / 9.807); // 2g scale in m/s^2
	az_f =((float) msg->imuZAccel) / (16384 / 9.807); // 2g scale in m/s^2

	gx_f=((float) msg->imuXGyro) / 131.072f; // for degrees/s 250 scale
	gy_f=((float) msg->imuYGyro) / 131.072f; // for degrees/s 250 scale
	gz_f=((float) msg->imuZGyro) / 131.072f; // for degrees/s 250 scale

	imu_msg.linear_acceleration.x=ax_f;
	imu_msg.linear_acceleration.y=ay_f;
	imu_msg.linear_acceleration.z=az_f;

	imu_msg.angular_velocity.x=gx_f;
	imu_msg.angular_velocity.y=gy_f;
	imu_msg.angular_velocity.z=gz_f;

	//calculate roll and pitch from accel values
	float pitch = 180 * atan(ax_f/sqrt(pow(ay_f,2) + pow(az_f,2)))/M_PI;
	float roll = 180 * atan(ay_f/sqrt(pow(ax_f,2) + pow(az_f,2)))/M_PI;

	//create Quaternion from roll and pitch
	tf::Quaternion quat2 = tf::createQuaternionFromRPY(roll,pitch,0);

	imu_msg.orientation.x=quat2.getX();
	imu_msg.orientation.y=quat2.getY();
	imu_msg.orientation.z=quat2.getZ();
	imu_msg.orientation.w=quat2.getW();

	imu_pub.publish(imu_msg);
}

IMU::~IMU(){
	std::cerr << "IMU object terminated" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_imu");

	IMU imu;

	ros::Rate loop_rate(30);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}


