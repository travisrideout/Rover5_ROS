#include <ros/ros.h>
#include <Rover5_ROS/Rover5Coms.h>
#include <Rover5_ROS/joy_teleop.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "rover5_ros");

	//ros::NodeHandle nHandle;

	Joy_TeleOp joy_teleop;

	ros::spin();
}
