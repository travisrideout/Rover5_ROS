/*
 * map.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: travis
 */

#include <Rover5_ROS/map.h>

Map::Map(){
}

void Map::Update_Map(){

	//Static map for now
	//pack up transform
	tf_quaternion = tf::createQuaternionFromYaw(0.0);
	tf::quaternionTFToMsg(tf_quaternion, msg_quaternion);
	map_tf.header.stamp = ros::Time::now(); //now;
	map_tf.header.frame_id = "map";
	map_tf.child_frame_id = "odom";
	map_tf.transform.translation.x = 0.0;
	map_tf.transform.translation.y = 0.0;
	map_tf.transform.translation.z = 0.0;
	map_tf.transform.rotation = msg_quaternion;

	//send the transform
	mapBroadcaster.sendTransform(map_tf);

}

Map::~Map(){
	std::cout << "Map object destroyed" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rover5_map");
	Map map;

	ros::Rate loop_rate(30);

	while(ros::ok()){
		map.Update_Map();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
