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

	bool GPU, show_lines, goal_received, crowdy, command_tail;

	float target_tolerance, minimal_step_dist, pos_update_threshold;

	float control_rate, cruise_linear_vel;

	float meas_linear_x, meas_angular_z, current_orientation;
	float prev_ref_linear_x, prev_meas_linear_x;
	float prev_ref_angular_z, prev_meas_angular_z;

	int time_sequence, averaged_ranges_size, state_sequence_size;
	std::pair<float, float> current_source;
	std::pair<float, float> current_target;
	
	std::pair<float, float> prev_source;
	std::pair<float, float> prev_target;
	std::pair<float, float> tmp_source;

	std::vector<float> range_data;

	std::string trained, online_model, logs_path;	

	std::string scan_topic, goal_topic, odom_topic, command_topic;

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


  };	


} // namespace neural_network_planner


#endif














