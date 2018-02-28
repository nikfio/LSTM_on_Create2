#ifndef _ONLINE_DEPLOY_H_
#define _ONLINE_DEPLOY_H_


// ROS related
#include <ros/ros.h>

#include <neural_network_planner/build_database.h>

// caffe related
#include <caffe/caffe.hpp>


// general 
#include <boost/thread.hpp>
#include <string>

/* if next yaw is within this limit cruise translational velocity is set
 * otherwise translational velocity is the minimum set
 */
const float CRUISE_ANGLE = 0.61056;


namespace neural_network_planner {


  class OnlineDeploy
  {

     public:
     
	OnlineDeploy(std::string& process_name);

	~OnlineDeploy();

     private:

	ros::NodeHandle private_nh;

     /*
	 * @brief predefined blobs to handle the training process
	 *        standard elements for every recurrent network
      */
	  
     boost::shared_ptr<caffe::Solver<float> > solver;
     boost::shared_ptr<caffe::Net<float> > online_net;

	boost::shared_ptr<caffe::Blob<float> > blobData;
	boost::shared_ptr<caffe::Blob<float> > blobClip;
	boost::shared_ptr<caffe::Blob<float> > blobLabel;
	boost::shared_ptr<caffe::Blob<float> > blobLoss;
	boost::shared_ptr<caffe::Blob<float> > blobOut;
	boost::shared_ptr<caffe::Blob<float> > blobArgmax;
	boost::shared_ptr<caffe::Blob<float> > blobSoftmax;

	bool GPU, show_lines, goal_received, steer_feedback, multiclass;

	float target_tolerance, minimal_step_dist, pos_update_threshold;

	float control_rate, cruise_vel, min_cruise_vel;
	
	float rotate_vel_res, min_rotate_vel, max_rotate_vel;
	int yaw_resolution;
	float yaw_measured, prev_yaw_measured;
	float prev_closest_steer, prev_yaw_ref;

	int out_size;
	float meas_linear_x, meas_angular_z;
	float prev_ref_linear_x, prev_meas_linear_x;
	float prev_ref_angular_z, prev_meas_angular_z;

	int time_sequence, averaged_ranges_size, state_sequence_size;
	std::pair<float, float> current_source;
	std::pair<float, float> current_target;
	
	std::pair<float, float> prev_source;
	std::pair<float, float> prev_target;
	std::pair<float, float> tmp_source;

	std::vector<float> range_data;
	
	std::vector<float> steer_angles;	

	std::string trained, online_model, logs_path;	

	std::string scan_topic, goal_topic, odom_topic, command_topic;
	std::string tail_type;

	visualization_msgs::Marker line_list;	

	message_filters::Subscriber<sensor_msgs::LaserScan> laserscan_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

	ros::Subscriber goal_sub_;
	ros::Publisher net_ranges_pub_;
	ros::Publisher marker_pub_;
	ros::Publisher vel_command_pub_;
	LaserScan state_ranges;

	float Step_dist();

	void updateTarget_callback(const MoveBaseActionGoal::ConstPtr& actiongoal_msg);

	void build_callback(const LaserScan::ConstPtr& laser_msg, 
					const Odometry::ConstPtr& odom_msg);

	void PublishZeroVelocity();

	inline float ComputeNewXPosition(float xi, float vx, float vy, float theta, float dt){
        return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
     }

	inline float ComputeNewYPosition(float yi, float vx, float vy, float theta, float dt){
        return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
     }

	inline float ComputeNewThetaPosition(float thetai, float vth, float dt){
        return thetai + vth * dt;
     }

	bool ComputeNewCommands(float yaw_ref, float yaw_measured, geometry_msgs::Twist& drive_cmds);

  };	


} // namespace neural_network_planner


#endif














