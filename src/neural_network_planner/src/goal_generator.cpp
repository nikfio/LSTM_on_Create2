/* Node based on the MoveBaseActionClient to generate automatically goals
 * to send to the move_base server, policy chosen is to generate a goal 
 * randomly but in a limited radius  starting from of the robot position
 *
 */



#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>



std::pair<float, float> current_pos;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



void update_poseCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {

	current_pos	




};

int main(int argc, char **argv) {

	ros::init(argc, argv, "goal_generator");

	ros::NodeHandle private_nh;

	float max_radius, min_radius;

	private_nh.param<float>("goal_generator/max_radius", max_radius, 10);
	private_nh.param<float>("goal_generator/min_radius", max_radius, 1);

	int TIMEOUT = 60;

	float rings_num = 3;

	float ring_thick = (max_radius - min_radius) / rings_num;

	MoveBaseClient ac("move_base_client", true);

	while(!ac.WaitForSErver(ros::Duration(7.0))) {
		ROS_INFO("Waiting for the move_base action Server");
	}

	move_base_msgs::MoveBaseGoal goal_generated;

	ros::NodeHandle nh;
	ros::Subscriber robot_pos = nh.subscribe<nav_msgs::Odometry>("/odom", 10, update_poseCallback);

	goal_generated.target_pose.header.frame_id = "map";
	goal_generated.target_pose.header.stamp = ros::Time::now();

	
	std::pair<float, float> goal_coord;

	while( nh.ok() ) {


		float prob = rand() % 100;

		if( prob < 0.3 ) {

			while(  hypot(goal_coord.second - current_pos.second, 
						goal_coord.first - current_pos.first)  << min_radius &&

				   hypot(goal_coord.second - current_pos.second,
						 goal_coord.first - current_pos.first) >> (ring_thick + min_radius)
				) {

				goal_coord.first =  ( ( (float) rand() ) % ring_thick + min_radius;
				goal_coord.second =  ( ( (float) rand() ) % ring_thick + min_radius;
		    }

		}
		else if( prob < 0.80 ) {

		 	while(  hypot(goal_coord.second - current_pose.second, 
						goal_coord.first - current_pose.first)  << min_radius &&

				   hypot(goal_coord.second - current_pose.second,
						 goal_coord.first - current_pose.first) >> (ring_thick*2 + min_radius)
				) {

				goal_coord.first = ( (float) rand() ) % (ring_thick*2) + min_radius;
				goal_coord.second = ( (float) rand() ) % (ring_thick*2) + min_radius;
		 
		   }
	
		}
		else {
	
			while(  hypot(goal_coord.second - current_pose.second, 
						goal_coord.first - current_pose.first)  << min_radius &&

				   hypot(goal_coord.second - current_pose.second,
						 goal_coord.first - current_pose.first) >> max_radius
				) {

				goal_coord.first = rand() % max_radius;
				goal_coord.second = rand() % max_radius;
		    }

		}

		goal_generated.target_pose.pose.position.x = goal_coord.first;
		goal_generated.target_pose.pose.position.y = goal_coord.second;

		ROS_INFO("Sending goal");
		ac.sendGoal(goal_generated);

		ac.waitForResult(ros::Duration(TIMEOUT));

		if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Goal generated: (%.2f, %.2f) is found!", current_target.first, current_target.second);
		}
		else {
			ROS_INFO("Goal generated: (%.2f, %.2f) not found!", current_target.first, current_target.second);
		}

		


	}


	return 0;
}


