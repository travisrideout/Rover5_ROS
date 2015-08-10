/*
 * joy_teleop.cpp
 *
 *  Created on: Feb 22, 2015
 *      Author: travis
 */
#include <Rover5_ROS/joy_teleop.h>

Joy_TeleOp::Joy_TeleOp(){
	// joystick setup
	joy_msg.axes.resize(GENERIC_JOY_NUM_AXES);
	joy_msg.buttons.resize(GENERIC_JOY_NUM_BUTTONS);

	//Publishers
	vel_pub = nh_.advertise<geometry_msgs::TwistWithCovariance>("rover_cmd_vel", 1);

	//Subscribers
	joy_sub_ = nh_.subscribe("joy", 10, &Joy_TeleOp::JoyCallback, this);

	std::cout << "Finished init of Joy_Teleop" << std::endl;
}

/*----------------------------------------------------------------------------
  joy function
 *----------------------------------------------------------------------------*/
void Joy_TeleOp::JoyCallback(const sensor_msgs::Joy& unordered_joy) {
	//ROS_INFO("JoyCallback");
	sensor_msgs::Joy msg = joy_msg;

#if defined USE_XBOX_CONTROLLER
	msg.axes[T_X_AXIS] 				= unordered_joy.axes[0];
	msg.axes[T_Y_AXIS] 				= unordered_joy.axes[1];
	msg.axes[T_Z_AXIS] 				= unordered_joy.axes[2] - unordered_joy.axes[5];
	msg.axes[R_X_AXIS] 				= unordered_joy.axes[4];
	msg.axes[R_Y_AXIS] 				= unordered_joy.buttons[5] - unordered_joy.buttons[4];
	msg.axes[R_Z_AXIS] 				= unordered_joy.axes[3];

	msg.buttons[BTN_START]			= unordered_joy.buttons[7];
	msg.buttons[BTN_BACK_SEL]		= unordered_joy.buttons[6];
	msg.buttons[BTN_A]				= unordered_joy.buttons[0];
	msg.buttons[BTN_B]				= unordered_joy.buttons[1];
	msg.buttons[BTN_X]				= unordered_joy.buttons[2];
	msg.buttons[BTN_Y]				= unordered_joy.buttons[3];
	msg.buttons[BTN_UP]				= unordered_joy.axes[7] > 0 ? 1 : 0;
	msg.buttons[BTN_DOWN]			= unordered_joy.axes[7] < 0 ? 1 : 0;
	msg.buttons[BTN_RIGHT]			= unordered_joy.axes[6] < 0 ? 1 : 0;
	msg.buttons[BTN_LEFT]			= unordered_joy.axes[6] > 0 ? 1 : 0;
	msg.buttons[BTN_LEFT_BACK]		= unordered_joy.buttons[4];
	msg.buttons[BTN_RIGHT_BACK]		= unordered_joy.buttons[5];
	msg.buttons[BTN_LEFT_THUMB]		= unordered_joy.buttons[9];
	msg.buttons[BTN_RIGHT_THUMB]	= unordered_joy.buttons[10];
#elif defined USE_SIXAXIS_CONTROLLER
	msg.axes[T_X_AXIS] 				= unordered_joy.axes[0];
	msg.axes[T_Y_AXIS] 				= unordered_joy.axes[1];
	msg.axes[T_Z_AXIS] 				= unordered_joy.axes[12] - unordered_joy.axes[13];
	msg.axes[R_X_AXIS] 				= unordered_joy.axes[2];
	msg.axes[R_Y_AXIS] 				= unordered_joy.axes[3];
	msg.axes[R_Z_AXIS] 				= unordered_joy.axes[14] - unordered_joy.axes[15];

	msg.buttons[BTN_START]			= unordered_joy.buttons[3];
	msg.buttons[BTN_BACK_SEL]		= unordered_joy.buttons[0];
	msg.buttons[BTN_A]				= unordered_joy.buttons[14];
	msg.buttons[BTN_B]				= unordered_joy.buttons[13];
	msg.buttons[BTN_X]				= unordered_joy.buttons[15];
	msg.buttons[BTN_Y]				= unordered_joy.buttons[12];
	msg.buttons[BTN_UP]				= unordered_joy.buttons[4];
	msg.buttons[BTN_DOWN]			= unordered_joy.buttons[6];
	msg.buttons[BTN_RIGHT]			= unordered_joy.buttons[5];
	msg.buttons[BTN_LEFT]			= unordered_joy.buttons[7];
	msg.buttons[BTN_LEFT_BACK]		= unordered_joy.buttons[10];
	msg.buttons[BTN_RIGHT_BACK]		= unordered_joy.buttons[11];
	msg.buttons[BTN_LEFT_THUMB]		= unordered_joy.buttons[1];
	msg.buttons[BTN_RIGHT_THUMB]	= unordered_joy.buttons[2];
#else
	msg = unordered_joy;
#endif

	// set joy_msg
	joy_msg = msg;
	SetTwist();
}

void Joy_TeleOp::SetTwist(){
	geometry_msgs::TwistWithCovariance twist;
	twist.twist.linear.x = (joy_msg.axes[T_Y_AXIS] + joy_msg.axes[R_X_AXIS])/2;
	twist.twist.angular.z = (joy_msg.axes[T_Y_AXIS] - joy_msg.axes[R_X_AXIS])/WIDTH;		//right hand rule: counter clock wise is positive

	//Publish Twist
	vel_pub.publish(twist);
}

Joy_TeleOp::~Joy_TeleOp() {
	std::cout << "Joy_TeleOp object terminated" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_joy");
	Joy_TeleOp joy_teleop;

	ros::Rate loop_rate(FREQUENCY);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}


