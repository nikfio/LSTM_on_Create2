
#ifndef _BUILD_DATABASE_H_
#define _BUILD_DATABASE_H_

// ROS related
#include <message_filters/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


#include <glog/logging.h>
#include <vector>
#include <string>

#include <boost/thread.hpp>


using std::vector;


using namespace sensor_msgs;
using namespace nav_msgs;
using namespace move_base_msgs;
using namespace message_filters; 
using namespace geometry_msgs; 


// limits set by the driver in use
const float linear_neg = -0.5;
const float linear_pos = 0.5;
const float angular_neg = -4.5;
const float angular_pos = 4.5; 

const float noise_level = 0.05; 

namespace neural_network_planner {


class BuildDatabase 
{

public:

	BuildDatabase(std::string& database_name);
	
	~BuildDatabase();

private:

	ros::NodeHandle private_nh;

	int averaged_ranges_size, scan_time_span;

	float target_tolerance;	

	std::string backend, logs_path, base_path;	

	std::string scan_topic, goal_topic, odom_topic, command_topic;
	std::string tail_type;

	int out_size;

	message_filters::Subscriber<sensor_msgs::LaserScan> laserscan_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

	float dist_preweight;

	ros::Subscriber goal_sub_;
	ros::Subscriber command_sub_;
	ros::Publisher net_ranges_pub_;
	ros::Publisher marker_pub_;
	LaserScan state_ranges;

	int set_size, batch_size, state_sequence_size;
	int db_writestep, timestep, database_counter;

	vector<double> steering_angles;
	
	vector<float> range_data, tail;

	std::pair<float, float> current_source;
	std::pair<float, float> current_target;
	
	std::pair<float, float> prev_source;
	std::pair<float, float> prev_target;
	std::pair<float, float> tmp_source;

	float current_orientation;
	float ref_linear_x, meas_linear_x, label_linear_x;
	float ref_angular_z, meas_angular_z, label_angular_z;;
	float minimal_step_dist, sampling_rate, pos_update_threshold;
	float prev_ref_linear_x, prev_meas_linear_x;
	float prev_ref_angular_z, prev_meas_angular_z;


	bool show_lines, command_measured, goal_received, crowdy;
	bool saturate_vel, command_feedback;

	ros::Time cmdvel_time;

	ros::NodeHandle db_nh;

	visualization_msgs::Marker line_list;	

	void build_callback(const LaserScan::ConstPtr& laser_msg, 
					const Odometry::ConstPtr& odom_msg);

	void updateTarget_callback(const MoveBaseActionGoal::ConstPtr& actiongoal_msg);

	void updateCmdVel_callback(const geometry_msgs::Twist::ConstPtr& cmdvel_msg);

	float Step_dist();

};

int saturate(float neg_lim, float pos_lim, float& value);

float point_distance(std::pair<float, float>& start_point, std::pair<float, float>& end_point); 



} // namespace neural_network_planner


#endif
