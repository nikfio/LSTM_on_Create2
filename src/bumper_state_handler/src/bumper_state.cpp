/* Node to warn if an obstacle is going to be hit
 * by analyzing the bumper messages
 */


#include <ros/ros.h>
#include <ca_msgs/Bumper.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>

using std::endl;

static int obstacle_detected = 0;

void bumperCallback(const ca_msgs::Bumper::ConstPtr& state) {
	
	
	if(state->is_light_left) {
		obstacle_detected++;
		LOG(INFO) << "LIGHT LEFT CENTER IS OBSTACLE signal value: " 
			 << state->light_signal_left << endl;
	}
	else {
		LOG(INFO) << "LIGHT LEFT signal value: " 
			 << state->light_signal_left << endl;
	}

	if(state->is_light_front_left) {
		obstacle_detected++;
		LOG(INFO) << "LIGHT FRONT LEFT IS OBSTACLE signal value: " 
			 << state->light_signal_front_left << endl;
	}
	else {
		LOG(INFO) << "LIGHT FRONT LEFT FRONT signal value: "
			 << state->light_signal_front_left << endl;
	}	 

	if(state->is_light_center_left) {
		obstacle_detected++;
		LOG(INFO) << "LIGHT CENTER LEFT IS OBSTACLE signal value: " 
			 << state->light_signal_center_left << endl;
	}
	else {
		LOG(INFO) << "LIGHT CENTER LEFT signal value: "
			 << state->light_signal_center_left << endl;
	}	 

	if(state->is_light_center_right) {
		obstacle_detected++;
		LOG(INFO) << "LIGHT CENTER RIGHT IS OBSTACLE signal value: " 
			 << state->light_signal_center_right << endl;
	}
	else {
		LOG(INFO) << "LIGHT CENTER RIGHT FRONT signal value: "
			 << state->light_signal_center_right << endl;
	}	 
	
	if(state->is_light_front_right) {
		obstacle_detected++;
		LOG(INFO) << "LIGHT FRONT RIGHT IS OBSTACLE signal value: " 
			 << state->light_signal_front_right << endl;
	}
	else {
		LOG(INFO) << "LIGHT FRONT RIGHT FRONT signal value: "
			 << state->light_signal_front_right << endl;
	}	 

	if(state->is_light_right) {
		obstacle_detected++;
		LOG(INFO) << "LIGHT RIGHT IS OBSTACLE signal value: " 
			 << state->light_signal_right << endl;
	}
	else {
		LOG(INFO) << "LIGHT RIGHT FRONT signal value: "
			 << state->light_signal_right << endl;
	}
	

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "bumper_state");

	ros::NodeHandle bumper_node;

	ros::Subscriber bumper_sub = bumper_node.subscribe("bumper", 5, bumperCallback);

	//ros::spin();

	ros::Publisher block_pub = bumper_node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Rate loop_rate(5);

	int obstacle_count = 0;

	while( ros::ok() ) {

		if( obstacle_detected > 0 ) {
			
			geometry_msgs::Twist block_msg;
			block_msg.linear.x = 0;
			block_msg.angular.z = 0;
			block_pub.publish(block_msg);

			ROS_INFO("OBSTACLE DETECTED --> cmd_vel zeroed");

			obstacle_detected = 0;

		}
	
		ros::spinOnce();
	
		loop_rate.sleep();

		obstacle_count++;
	
	}

	return 0;

}
