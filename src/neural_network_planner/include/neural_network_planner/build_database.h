
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


namespace neural_network_planner {


class BuildDatabase 
{

public:

	BuildDatabase(std::string& database_name);
	
	~BuildDatabase();

private:

	ros::NodeHandle private_nh;

	int averaged_ranges_size, scan_time_span;	

	std::string backend, logs_path, base_path;	

	std::string scan_topic, goal_topic, odom_topic, command_topic;

	message_filters::Subscriber<sensor_msgs::LaserScan> laserscan_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

	ros::Subscriber goal_sub_;
	ros::Subscriber command_sub_;
	ros::Publisher net_ranges_pub_;
	ros::Publisher marker_pub_;
	LaserScan state_ranges;

	int set_size, batch_size, state_sequence_size;
	int db_writestep, timestep, database_counter;
	
	double move_angle_distance;

	vector<double> steering_angles;
	
	vector<float> range_data;

	std::pair<float, float> current_source;
	std::pair<float, float> current_target;
	
	std::pair<float, float> prev_source;
	std::pair<float, float> prev_target;
	std::pair<float, float> tmp_source;

	float current_orientation;
	float current_linear_x;
	float current_angular_z;
	float minimal_step_dist, sampling_rate, pos_update_threshold;

	bool show_lines, command_measured, goal_received;

	ros::Time cmdvel_time;

	ros::NodeHandle db_nh;

	visualization_msgs::Marker line_list;	

	void build_callback(const LaserScan::ConstPtr& laser_msg, 
					const Odometry::ConstPtr& odom_msg);

	void updateTarget_callback(const MoveBaseActionGoal::ConstPtr& actiongoal_msg);

	void updateCmdVel_callback(const geometry_msgs::Twist::ConstPtr& cmdvel_msg);

	float Step_dist();

};

float point_distance(std::pair<float, float>& start_point, std::pair<float, float>& end_point); 



} // namespace neural_network_planner


#endif
