/* Node based on the MoveBaseActionClient to generate automatically goals
 * to send to the MoveBaseServer running in the move_base node, policies: 
 * 	1) randomly but in a limited radius  starting from of the robot position
 *	2) user launching this node specfies goal parameters relative to currrent robot position
 */



#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <tf/transform_datatypes.h>

#include <neural_network_planner/build_database.h>

#include <glog/logging.h>

using neural_network_planner::saturate;

std::pair<float, float> current_pos;
float yaw_measured;

void update_poseCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {

	LOG(INFO) << "Update Position";
	current_pos.first = odom_msg->pose.pose.position.x;
	current_pos.second = odom_msg->pose.pose.position.y;
	tf::Pose pose;
     tf::poseMsgToTF(odom_msg->pose.pose, pose);
     yaw_measured = tf::getYaw(pose.getRotation());

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "goal_generator");

	ros::NodeHandle private_nh;

	int max_radius, min_radius;
	float one_angle, one_dist;
	bool one_shot;
	int timeout;

	private_nh.param<int>("max_radius", max_radius, 10);
	private_nh.param<int>("min_radius", max_radius, 1);
	private_nh.param("one_shot", one_shot, true);
	private_nh.param<float>("one_angle", one_angle, M_PI_2);
	private_nh.param<float>("one_dist", one_dist, 2);
	private_nh.param<int>("timeout", timeout, 120);
 
	
	float rings_num = 3;

	int ring_thick = (max_radius - min_radius) / rings_num;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(7.0))) {
		LOG(INFO) << "Waiting for the move_base action Server";
	}

	move_base_msgs::MoveBaseGoal goal_generated;

	ros::NodeHandle nh;
	ros::Subscriber robot_pos = nh.subscribe<nav_msgs::Odometry>("/odom", 1, update_poseCallback);

	goal_generated.target_pose.header.frame_id = "map";
	goal_generated.target_pose.header.stamp = ros::Time::now();

	std::pair<float, float> goal_coord;

	while(current_pos.first == 0 || current_pos.second == 0) {
		ros::spinOnce();
	}

	ros::Rate min_rate(0.2);

	while( nh.ok() ) { 

     
	   LOG(INFO) << "Current Position update: (" << current_pos.first 
				<< "," << current_pos.second << ")";	

			char mode;
			LOG(INFO) << "Coordinates or distance - angle? (c/d)";
			std::cin >> mode;
			if( mode == 'c' ) {
				float new_x, new_y;
				LOG(INFO) << "Enter x coordinate: ";
				std::cin >> new_x;
				LOG(INFO) << "Enter y coordinate: ";
				std::cin >> new_y;											
				
				goal_coord.first = current_pos.first + new_x;
				goal_coord.second = current_pos.second + new_y;	

			}
			else if( mode == 'd' ) {
				float dist, angle;
				LOG(INFO) << "Enter relative distance: ";
				std::cin >> dist;
				LOG(INFO) << "Enter relative angle [-180, +180]: ";
				std::cin >> angle;	

				float yaw_req = angle * M_PI / 180;

				
				float sat_res = saturate( YAW_NEG_LIM, YAW_POS_LIM, yaw_req);

				if( sat_res == YAW_NEG_LIM ) {
					
					yaw_req = YAW_POS_LIM - fabs(YAW_NEG_LIM - yaw_req);
					goal_coord.first = current_pos.first + dist * cos(yaw_req);
					goal_coord.second = current_pos.second + dist * sin(yaw_req);	

				}
				else if( sat_res == YAW_POS_LIM ) {
			
					yaw_req = YAW_NEG_LIM + (YAW_POS_LIM - yaw_req);
					goal_coord.first = current_pos.first + dist * cos(yaw_req);
					goal_coord.second = current_pos.second + dist * sin(yaw_req);

				}
				else {
			
					goal_coord.first = current_pos.first + dist * cos(yaw_measured + angle);
					goal_coord.second = current_pos.second + dist * sin(yaw_measured + angle);
	
				}
				
				LOG(INFO) << "YAW DATA: " << yaw_measured << "   " << (yaw_measured + yaw_req) << "  " << yaw_req;

			}
			else {

				LOG(ERROR) << "NO VALID ANSWER!"; 
				continue;

			}
		
		goal_generated.target_pose.pose.position.x = goal_coord.first;
		goal_generated.target_pose.pose.position.y = goal_coord.second;
		goal_generated.target_pose.pose.orientation.w = 1.0;
	
		ROS_INFO("Sending goal: ( %.3f , %.3f ) ", goal_coord.first, goal_coord.second);
		ac.sendGoal(goal_generated);

		ros::spinOnce();

		bool time_check = ac.waitForResult(ros::Duration(timeout));

		if (time_check) {
			if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				LOG(INFO)  << "Goal generated : ( " << goal_coord.first 
						 << " , " << goal_coord.second << " ) - FOUND!";
			}
			else {
				LOG(INFO)  << "Goal generated : ( " << goal_coord.first 
						 << " , " << goal_coord.second << " ) - NOT FOUND!";
			}
		}
		else {

			LOG(INFO) << "No result within the timeout";
		}

		ros::spinOnce();

		min_rate.sleep();

	}



	return 0;
}


