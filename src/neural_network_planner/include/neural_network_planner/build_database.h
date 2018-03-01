
#ifndef _BUILD_DATABASE_H_
#define _BUILD_DATABASE_H_

// ROS related
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

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
const float TRASL_MAX = 0.5;
const float ROTATE_MAX = 4.5; 

const float noise_level = 0.05; 

namespace neural_network_planner {


class BuildDatabase 
{

public:

	BuildDatabase(std::string& database_name);
	
	~BuildDatabase();

private:

	ros::NodeHandle private_nh;

	int averaged_ranges_size;

	float target_tolerance;	

	std::string backend, logs_path, base_path;	

	std::string scan_topic, goal_topic, odom_topic;
	std::string tail_type;

	message_filters::Subscriber<sensor_msgs::LaserScan> laserscan_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

	float dist_scale, yaw_scale;

	ros::Subscriber goal_sub_;
	ros::Publisher net_ranges_pub_;
	ros::Publisher marker_pub_;
	LaserScan state_ranges;

	int set_size, batch_size, state_sequence_size;
	int db_writestep, timestep, database_counter;

	vector<float> steer_angles;
	
	vector<float> range_data;

	std::pair<float, float> current_source;
	std::pair<float, float> current_target;
	
	std::pair<float, float> prev_source;
	std::pair<float, float> prev_target;
	std::pair<float, float> tmp_source;
	
	float step_resolution, sampling_rate, pos_update_threshold;
	float meas_linear_x, meas_angular_z;
	float ref_linear_x, ref_angular_z;

	float yaw_measured, prev_yaw_measured;
	float prev_closest_steer, prev_real_steer;
	int steer_resolution, min_steer_angle, max_steer_angle;
	
	bool goal_received, steer_feedback;	

	ros::Time cmdvel_time;

	ros::NodeHandle db_nh;	

	void build_callback(const LaserScan::ConstPtr& laser_msg, 
					const Odometry::ConstPtr& odom_msg);

	void updateTarget_callback(const MoveBaseActionGoal::ConstPtr& actiongoal_msg);

	void updateCmdVel_callback(const geometry_msgs::Twist::ConstPtr& cmdvel_msg);

	float Step_dist();

};

int saturate(float neg_lim, float pos_lim, float& value);

float point_distance(std::pair<float, float>& start_point, std::pair<float, float>& end_point);

void initializeSteer(vector<float>& steer_angles, int steer_resolution, int min_angle, int max_angle);
	
int getClosestSteer(vector<float>& steer_angles, float yaw_angle, float& closest ); 
 



} // namespace neural_network_planner


#endif
