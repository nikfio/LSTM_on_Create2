
// ROS related
#include <neural_network_planner/train_validate.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// caffe related

#include "glog/logging.h"

#include <boost/lexical_cast.hpp>

#include <ctime>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <cstdlib>


#include <neural_network_planner/online_deploy.h>

using visualization_msgs::Marker;

using std::cout;
using std::endl;
using std::cin;
using std::string;

using boost::lexical_cast;



namespace neural_network_planner {

  OnlineDeploy::OnlineDeploy(string& process_name) : private_nh("~")
  {
	
	private_nh.param("trained", trained, std::string("") );
	private_nh.param("online_model", online_model, std::string("") );
	private_nh.param("GPU", GPU, true );
	private_nh.param("averaged_ranges_size", averaged_ranges_size, 22);
	private_nh.param("time_sequence", time_sequence, 22);	
	private_nh.param<float>("target_tolerance", target_tolerance, 0.08);
	private_nh.param("scan_topic", scan_topic, std::string("/base_scan") );
	private_nh.param("goal_topic", goal_topic, std::string("/move_base/goal") );	
	private_nh.param("odom_topic", odom_topic, std::string("/odom") );
	private_nh.param("command_topic", command_topic, std::string("/cmd_vel") );
	private_nh.param<float>("pos_update_threshold", pos_update_threshold, 0.001);
	private_nh.param("show_lines", show_lines, false);
	private_nh.param("crowdy", crowdy, false); 
	private_nh.param("logs_path", logs_path, std::string(""));
	private_nh.param<float>("control_rate", control_rate, 5);
	private_nh.param<float>("cruise_linear_vel", cruise_linear_vel, 0.3);
	private_nh.param("command_feedback", command_feedback, true);
	private_nh.param<int>("output_size", out_size, 1);
	private_nh.param("feedback_type", tail_type, std::string(""));

	FLAGS_log_dir = logs_path;
	FLAGS_alsologtostderr = 1;
	FLAGS_minloglevel = 0;

	prev_target = std::pair<float, float>(0,0);
	current_source = std::pair<float,float>(0,0);
	prev_source = std::pair<float,float>(0,0);
	
	range_data = vector<float>(averaged_ranges_size, 0);


	if (GPU) {
		caffe::Caffe::set_mode(caffe::Caffe::GPU);
		LOG(INFO) << "GPU mode"; 
	} else {
		caffe::Caffe::set_mode(caffe::Caffe::CPU);
		LOG(INFO) << "CPU mode";
	}

	FLAGS_minloglevel = 1;

	LOG(WARNING) << "Loading " << online_model;

	// initialize net for online test
	online_net.reset(new caffe::Net<float>(online_model, caffe::TEST));
	online_net->CopyTrainedLayersFrom(trained);

	CHECK(online_net->has_blob("data"));	
	CHECK(online_net->has_blob("out"));	
						
	blobData = online_net->blob_by_name("data");
	blobClip = online_net->blob_by_name("clip");
	blobOut = online_net->blob_by_name("out");

	// input state size checks
	if( command_feedback && out_size == 1 ) 
		state_sequence_size = averaged_ranges_size + 3;
	else if( command_feedback && out_size == 2 ) 
		state_sequence_size = averaged_ranges_size + 4;
	else
		state_sequence_size = averaged_ranges_size + 2;

	CHECK_EQ(state_sequence_size, blobData->shape(1) ) << "net: supposed input sequence size check failed";

	CHECK_EQ(out_size, blobOut->shape(1) ) << " supposed output size and net loaded output must equal";

	time_t now = time(0);
	tm *local = localtime(&now);

	string online_test = "online_test" + online_net->name() + "_" + lexical_cast<std::string>(local->tm_mon) 
				     + "-" + lexical_cast<std::string>(local->tm_mday) + "-" + lexical_cast<std::string>(local->tm_hour)
				     + "_" + lexical_cast<std::string>(local->tm_min);

	ros::NodeHandle online_nh("online_deploy");
	laserscan_sub_.subscribe(online_nh, scan_topic, 25);
	odom_sub_.subscribe(online_nh, odom_topic, 25);

	typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> StepPolicy;
	
	Synchronizer<StepPolicy> step_sync( StepPolicy(10), laserscan_sub_, odom_sub_);
	step_sync.registerCallback(boost::bind(&neural_network_planner::OnlineDeploy::build_callback, this, _1, _2));

	ros::NodeHandle nh;
	goal_sub_ = nh.subscribe<MoveBaseActionGoal>(goal_topic , 1, 
						boost::bind(&neural_network_planner::OnlineDeploy::updateTarget_callback, this, _1));
	
	vel_command_pub_ = nh.advertise<geometry_msgs::Twist>(command_topic , 1);

	net_ranges_pub_ = nh.advertise<LaserScan>("state_ranges", 1);
	if( show_lines ) {
		marker_pub_ = nh.advertise<Marker>("range_lines", 1);
	}

	bool measured;
	if( tail_type == "measured" ) 
		measured = true;
	else if ( tail_type == "reference" )
		measured = false;
	else 
		LOG(FATAL) << "command feedback type not valid {measured, reference}";

	goal_received = false;

	ros::Rate store_rate(control_rate);

	int online_step = 0;

	FLAGS_minloglevel = 0;

	LOG(INFO) << "ONLINE NET INITIALIZED ";

	while( ros::ok() ) {

//	  LOG(INFO) << "Distance from previous point: " << Step_dist() << "   " << goal_received;	  
		
	  if( goal_received ) { 


		// populate clip blob 
		if( online_step % time_sequence == 0 ) {
			for(int j = 0; j < state_sequence_size; j++) {
				blobClip->mutable_cpu_data()[j] = 0;
			}
		}
		else {
			for(int j = 0; j < state_sequence_size; j++) {
				blobClip->mutable_cpu_data()[j] = 1;
			}
		}


		float x_rel = current_target.first - current_source.first;
		float y_rel = current_target.second - current_source.second;	 
		
		// load the state as net input 				
		for(int i = 0; i < range_data.size(); i++) {
			blobData->mutable_cpu_data()[i] = range_data[i];
		}	
	
		blobData->mutable_cpu_data()[range_data.size()] = hypot( x_rel, y_rel);
		blobData->mutable_cpu_data()[range_data.size() + 1] =
					 fabs(atan2( y_rel , x_rel ) - current_orientation);

		if(out_size == 1) {
			if( command_feedback && measured ) {
				blobData->mutable_cpu_data()[range_data.size() + 2] = prev_meas_angular_z;
	 		}
			else if( command_feedback && !measured ) {
				blobData->mutable_cpu_data()[range_data.size() + 2] = prev_ref_angular_z;
	 		}
		}
		else if(out_size == 2) {
			if( command_feedback && measured ) {
				blobData->mutable_cpu_data()[range_data.size() + 2] = prev_meas_linear_x;
				blobData->mutable_cpu_data()[range_data.size() + 3] = prev_meas_angular_z;
	 		}
			else if( command_feedback && !measured ) {
				blobData->mutable_cpu_data()[range_data.size() + 2] = prev_ref_linear_x;
				blobData->mutable_cpu_data()[range_data.size() + 3] = prev_ref_angular_z;
	 		}
		}


//		for(int i = 0; i < state_sequence_size; i++) {
//			cout << blobData->mutable_cpu_data()[i] << "  ";
//		}

		online_net->Forward();
	
		geometry_msgs::Twist cmd_out;

		float linear;
		float angular;

		if(out_size == 1) {
			linear = cruise_linear_vel;
			angular = blobOut->mutable_cpu_data()[0];
		}
		else if(out_size == 2) {
			linear = blobOut->mutable_cpu_data()[0];
			angular = blobOut->mutable_cpu_data()[1];
		}


		LOG(INFO) << "Linear: " <<  linear << " Angular: " << angular;

		if( saturate(linear_neg, linear_pos, linear) ) {
			LOG(INFO) << "Linear velocity command saturating";
		}

		if( saturate(angular_neg, angular_pos, angular) ) {
			LOG(INFO) << "Angular velocity command saturating";
		}	

		cmd_out.linear.x = linear;
		cmd_out.angular.z = angular;	
		
		vel_command_pub_.publish(cmd_out);
		
		online_step++;

		prev_ref_angular_z = angular;
		prev_ref_linear_x = linear;
		prev_meas_angular_z = meas_angular_z; 
		prev_meas_linear_x = meas_linear_x; 	

      }

	 if ( point_distance(current_source, current_target) <= target_tolerance ) {
		 // current pos has achieved target within the tolerance
//		LOG(INFO) << "target reached";
		PublishZeroVelocity();
		goal_received = false;
	 }

	 prev_source = current_source;

	 store_rate.sleep();
	
	 ros::spinOnce();

  }



}

 OnlineDeploy::~OnlineDeploy() 
 {



 }


 float OnlineDeploy::Step_dist() {

	return hypot(current_source.second - prev_source.second, current_source.first - prev_source.first);

 }

	
 void OnlineDeploy::updateTarget_callback( const MoveBaseActionGoal::ConstPtr& actiongoal_msg) {

	LOG(INFO) << "Updating target pose";
	current_target.first = actiongoal_msg->goal.target_pose.pose.position.x;
	current_target.second = actiongoal_msg->goal.target_pose.pose.position.y;
	goal_received = true;

 } 

 /* this thread is collecting navigation data - first variant - labels are set depending
 * on the real path effectively taken by the robot - output of controller (local planner)
 */
void OnlineDeploy::build_callback(const LaserScan::ConstPtr& laser_msg, 
							const Odometry::ConstPtr& odom_msg)
{
	
	// update measured robot position
	tmp_source.first = odom_msg->pose.pose.position.x;
	tmp_source.second = odom_msg->pose.pose.position.y;
	current_orientation = odom_msg->pose.pose.orientation.z;

	if ( point_distance(current_source, tmp_source) > pos_update_threshold ) 
		current_source = tmp_source;

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


	CHECK_EQ(current_ranges.size(), averaged_ranges_size);

	for(int i = 0; i < current_ranges.size(); i++) {
		range_data[i] = current_ranges[i];
	}
	
		
	float start_angle = (float) (laser_msg->angle_max - laser_msg->angle_min) / (averaged_ranges_size*2);
 
	state_ranges.angle_min = laser_msg->angle_min + start_angle;
	state_ranges.angle_max = laser_msg->angle_max;
	state_ranges.range_min = laser_msg->range_min;
	state_ranges.range_max = laser_msg->range_max;
	state_ranges.header.stamp = line_list.header.stamp = ros::Time::now();
	state_ranges.ranges = range_data;
	state_ranges.angle_increment = (state_ranges.angle_max - state_ranges.angle_min) / averaged_ranges_size;
	state_ranges.header.frame_id = "hokuyo_link";
	line_list.header.frame_id = "/odom";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.color.r = 1.0;
     line_list.color.a = 1.0;

	geometry_msgs::Point p_source;
	p_source.x = tmp_source.first;
	p_source.y = tmp_source.second;

	if( show_lines ) {

		for(int i = 0; i < current_ranges.size(); i++) {

			geometry_msgs::Point p;
			p.x = p_source.x + current_ranges[i] * cos(state_ranges.angle_min + state_ranges.angle_increment * i);
			p.y = p_source.y + current_ranges[i] * sin(state_ranges.angle_min + state_ranges.angle_increment * i);
			p.z = 0.025;

			line_list.points.push_back(p);
			line_list.points.push_back(p_source);

		}
		
		marker_pub_.publish(line_list);
	}

	net_ranges_pub_.publish(state_ranges);	

}


void OnlineDeploy::PublishZeroVelocity() {

	geometry_msgs::Twist zero_vel;
	zero_vel.linear.x = 0;
	zero_vel.angular.z = 0;
	
	vel_command_pub_.publish(zero_vel);


}



} // namespace neural_network_planner





