#include <ros/ros.h>
#include <Rover5Coms.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rover5_ros");

	ros::spin();
}
