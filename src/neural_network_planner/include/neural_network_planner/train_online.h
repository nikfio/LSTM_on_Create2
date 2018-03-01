
#ifndef _TRAIN_VALIDATE_RNN_ 
#define _TRAIN_VALIDATE_RNN_

// ROS related
#include <ros/ros.h>
#include <neural_network_planner/build_database.h>

// caffe related
#include <caffe/caffe.hpp>


// general 
#include <boost/thread.hpp>
#include <string>




namespace neural_network_planner {


  class TrainOnline 
  {

     public:
     
	TrainOnline(std::string& process_name);

	~TrainOnline();

     private:

	ros::NodeHandle private_nh;

     /*
	 * @brief predefined blobs to handle the training process
	 *        standard elements for every recurrent network
      */
	 

     boost::shared_ptr<caffe::Solver<float> > solver;
     boost::shared_ptr<caffe::Net<float> > net;
	boost::shared_ptr<caffe::Net<float> > test_net;

	boost::shared_ptr<caffe::Blob<float> > blobData_;
	boost::shared_ptr<caffe::Blob<float> > blobLabel_;

	boost::shared_ptr<caffe::Blob<float> > blobData;
	boost::shared_ptr<caffe::Blob<float> > blobClip;
	boost::shared_ptr<caffe::Blob<float> > blobLabel;
	boost::shared_ptr<caffe::Blob<float> > blobLoss;
	boost::shared_ptr<caffe::Blob<float> > blobOut;
	boost::shared_ptr<caffe::Blob<float> > blobArgmax;
	boost::shared_ptr<caffe::Blob<float> > blobAccu;
	boost::shared_ptr<caffe::Blob<float> > blobSoftmax;


	boost::shared_ptr<caffe::Blob<float> > test_blobData;
	boost::shared_ptr<caffe::Blob<float> > test_blobClip;
	boost::shared_ptr<caffe::Blob<float> > test_blobLabel;
	boost::shared_ptr<caffe::Blob<float> > test_blobLoss;
	boost::shared_ptr<caffe::Blob<float> > test_blobOut;
     boost::shared_ptr<caffe::Blob<float> > test_blobAccu;
	boost::shared_ptr<caffe::Blob<float> > test_blobArgmax;
	boost::shared_ptr<caffe::Blob<float> > test_blobSoftmax;

	std::string solver_conf, trained, folder_path, logs_path;

	bool TRAIN, GPU, resume, steer_feedback;
	bool multiclass, online_training;

	std::vector<float> steer_angles;

	int steer_resolution, minibatch, total_iterations;

	int batch_updates, iter_size, val_freq;
	int state_sequence_size;
	int time_sequence, averaged_ranges_size;

	std::string scan_topic, goal_topic, odom_topic;

	message_filters::Subscriber<sensor_msgs::LaserScan> laserscan_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

	int dist_scale, yaw_scale, checking_rate;

	ros::Subscriber goal_sub_;
	ros::Publisher net_ranges_pub_;
	ros::Publisher marker_pub_;
	sensor_msgs::LaserScan state_ranges;

	std::vector<float> range_data;

	std::pair<float, float> current_source;
	std::pair<float, float> current_target;
	
	std::pair<float, float> prev_source;
	std::pair<float, float> prev_target;
	std::pair<float, float> tmp_source;
	
	float step_resolution, pos_update_threshold;
	float meas_linear_x, meas_angular_z;
	float ref_linear_x, ref_angular_z;

	int min_steer_angle, max_steer_angle;

	float yaw_measured, prev_yaw_measured, prev_closest_steer;
	float target_tolerance;
	
	bool goal_received;	

	void build_callback(const LaserScan::ConstPtr& laser_msg, 
					const Odometry::ConstPtr& odom_msg);

	void updateTarget_callback(const MoveBaseActionGoal::ConstPtr& actiongoal_msg);

	void updateCmdVel_callback(const geometry_msgs::Twist::ConstPtr& cmdvel_msg);

	void UpdateTestNet();

	float Step_dist();

  };



} // namespace neural_network_planner


#endif

