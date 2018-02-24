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

#include <glog/logging.h>


std::pair<float, float> current_pos;


void update_poseCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {

	LOG(INFO) << "Update Position";
	current_pos.first = odom_msg->pose.pose.position.x;
	current_pos.second = odom_msg->pose.pose.position.y;

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
	std::pair<float, float> current_target;

	ros::spinOnce();

	ros::Duration(2).sleep();

	if( !one_shot ) {

	while( nh.ok() ) {  // random goal gen cycle


		float prob = rand() % 100;

		if( prob < 0.3 ) {

			while(  hypot(goal_coord.second - current_pos.second, 
						goal_coord.first - current_pos.first)  < min_radius &&

				   hypot(goal_coord.second - current_pos.second,
						 goal_coord.first - current_pos.first) > (ring_thick + min_radius)
				) 
		    {

				goal_coord.first =    rand()  % ring_thick + min_radius;
				goal_coord.second =   rand()  % ring_thick + min_radius;
		    }

		}
		else if( prob < 0.80 ) {

		 	while(  hypot(goal_coord.second - current_pos.second, 
						goal_coord.first - current_pos.first)  < min_radius &&

				   hypot(goal_coord.second - current_pos.second,
						 goal_coord.first - current_pos.first) > (ring_thick*2 + min_radius)
				)
      	    {

				goal_coord.first = rand()  % (ring_thick*2)  + min_radius;
				goal_coord.second = rand()  % (ring_thick*2)  + min_radius;
		 
		    }
	
		}
		else {
	
			while(  hypot(goal_coord.second - current_pos.second, 
						goal_coord.first - current_pos.first)  < min_radius &&

				   hypot(goal_coord.second - current_pos.second,
						 goal_coord.first - current_pos.first) > max_radius
				) {

				goal_coord.first =  rand() % max_radius;
				goal_coord.second = rand() % max_radius;
		    }

		}

		goal_generated.target_pose.pose.position.x = goal_coord.first;
		goal_generated.target_pose.pose.position.y = goal_coord.second;
		goal_generated.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending goal");
		ac.sendGoal(goal_generated);

		bool time_check = ac.waitForResult(ros::Duration(timeout));

		if (time_check) {
			if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				LOG(INFO)  << "Goal generated : ( " << current_target.first  
						 << " , " << current_target.second << " ) - FOUND!";
			}
			else {
				LOG(INFO)  << "Goal generated : ( " << current_target.first  
						 << " , " << current_target.second << " ) - NOT FOUND!";
			}
		}
		else {

			LOG(INFO) << "No result within the timeout";
		}

	} // random goal gen cycle end		


	} 
	else { // user definded goal cycle

		LOG(INFO) << "Current Position update: (" << current_pos.first 
				<< "," << current_pos.second << ")";	

		char answer;
		LOG(INFO) << "New target coordinates? (y/n)";
		std::cin >> answer;
		
		if(answer == 'y' ) {

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
				LOG(INFO) << "Enter relative angle: ";
				std::cin >> angle;	

				goal_coord.first = current_pos.first + dist * cos(angle);
				goal_coord.second = current_pos.second + dist * sin(angle);	
										
			}
			else {

				LOG(INFO) << "Default: dist - angle from yaml config"; 
			
				goal_coord.first = current_pos.first + one_dist * cos(one_angle);
				goal_coord.second = current_pos.second + one_dist * sin(one_angle);	

			}
		}
		else {
				LOG(INFO) << "Default: dist - angle from yaml config"; 
			
				goal_coord.first = current_pos.first + one_dist * cos(one_angle);
				goal_coord.second = current_pos.second + one_dist * sin(one_angle);	
		}

		goal_generated.target_pose.pose.position.x = goal_coord.first;
		goal_generated.target_pose.pose.position.y = goal_coord.second;
		goal_generated.target_pose.pose.orientation.w = 1.0;
	

		ROS_INFO("Sending goal: ( %.3f , %.3f ) ", goal_coord.first, goal_coord.second);
		ac.sendGoal(goal_generated);

		bool time_check = ac.waitForResult(ros::Duration(timeout));

		if (time_check) {
			if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				LOG(INFO)  << "Goal generated : ( " << current_target.first 
						 << " , " << current_target.second << " ) - FOUND!";
			}
			else {
				LOG(INFO)  << "Goal generated : ( " << current_target.first 
						 << " , " << current_target.second << " ) - NOT FOUND!";
			}
		}
		else {

			LOG(INFO) << "No result within the timeout";
		}

	}  // user defined goal cycle end



	return 0;
}


