/* Functions to build the batch for online training */

#include <neural_network_planner/train_validate.h>
#include <neural_network_planner/build_database.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace move_base_msgs;
using namespace message_filters; 
using namespace geometry_msgs; 

using std::cout;
using std::endl;

namespace neural_network_planner {



void TrainValidateRNN::build_callback(const LaserScan::ConstPtr& laser_msg, 
							const Odometry::ConstPtr& odom_msg)
{
	
	
	// update measured robot position
	tmp_source.first = odom_msg->pose.pose.position.x;
	tmp_source.second = odom_msg->pose.pose.position.y;


	if ( point_distance(current_source, tmp_source) > pos_update_threshold ) 
		current_source = tmp_source;

	tf::Pose pose;
     tf::poseMsgToTF(odom_msg->pose.pose, pose);
     yaw_measured = tf::getYaw(pose.getRotation());

	meas_linear_x = odom_msg->twist.twist.linear.x;
	meas_angular_z = odom_msg->twist.twist.angular.z;
	
	vector<float> ranges = laser_msg->ranges;
	
	float range_num = ranges.size();
	vector<float> current_ranges;
	
	int average_base = range_num / averaged_ranges_size;

	float add_range;	
		

	for(int i = 0; i < range_num; i++) {

		add_range += ranges[i];

		if( (i+1) % average_base == 0 ) {
			current_ranges.push_back( add_range / average_base );
			add_range = 0;
		}
			
	}		


	for(int i = 0; i < current_ranges.size(); i++) {
		range_data[i] = current_ranges[i];

	}
	
		
	float start_angle = (float) (laser_msg->angle_max - laser_msg->angle_min) / (averaged_ranges_size*2);
 
	state_ranges.angle_min = laser_msg->angle_min + start_angle;
	state_ranges.angle_max = laser_msg->angle_max;
	state_ranges.range_min = laser_msg->range_min;
	state_ranges.range_max = laser_msg->range_max;
	state_ranges.header.stamp = ros::Time::now();
	state_ranges.ranges = range_data;
	state_ranges.angle_increment = (state_ranges.angle_max - state_ranges.angle_min) / averaged_ranges_size;
	state_ranges.header.frame_id = "hokuyo_link";
	

	geometry_msgs::Point p_source;
	p_source.x = tmp_source.first;
	p_source.y = tmp_source.second;

	net_ranges_pub_.publish(state_ranges);	

}


void TrainValidateRNN::updateTarget_callback( const MoveBaseActionGoal::ConstPtr& actiongoal_msg) {

//	LOG(INFO) << "Updating target pose";
	current_target.first = actiongoal_msg->goal.target_pose.pose.position.x;
	current_target.second = actiongoal_msg->goal.target_pose.pose.position.y;
	goal_received = true;

}

float TrainValidateRNN::Step_dist() {

	return hypot(current_source.second - prev_source.second, current_source.first - prev_source.first);

}

} // namespace neural_network_planner
