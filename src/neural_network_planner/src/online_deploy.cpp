
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
	private_nh.param("logs_path", logs_path, std::string(""));
	private_nh.param<float>("control_rate", control_rate, 5);
	private_nh.param<float>("cruise_vel", cruise_vel, 0.4);
	private_nh.param<float>("min_cruise_vel", min_cruise_vel, 0.2);
	private_nh.param("steer_feedback", command_feedback, true);
	private_nh.param("multiclass", multiclass, true);
	private_nh.param("yaw_resolution", yaw_resolution, 10); 
	private_nh.param<float>("rotate_vel_res", rotate_vel_res, 0.1);
	private_nh.param<float>("min_rotate_vel", min_rotate_vel, 0.1);
	private_nh.param<float>("max_rotate_vel", max_rotate_vel, 1);
	
 	CHECK_EQ( (max_rotate_vel - min_rotate_vel) % rotate_vel_res, 0);

	CHECK_LE( max_rotate_vel, ROTATE_MAX ) << " max_rotate_vel limit exceeded"; 
	CHECK_LE( cruise_trasl_vel, TRASL_MAX ) << " cruise_trasl_vel limit exceeded";

	FLAGS_log_dir = logs_path;
	FLAGS_alsologtostderr = 1;
	FLAGS_minloglevel = 0;

	prev_target = std::pair<float, float>(0,0);
	current_source = std::pair<float,float>(0,0);
	prev_source = std::pair<float,float>(0,0);
	
	prev_closest_steer = prev_yaw_ref =  0
	
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
	if( multiclass ) {
		CHECK(net->has_blob("argmax"));
	}		
						
	blobData = online_net->blob_by_name("data");
	blobClip = online_net->blob_by_name("clip");
	blobOut = online_net->blob_by_name("out");
	blobArgmax = online_net->blob_by_name("argmax");

	// input state size checks
	if( steer_feedback ) 
		state_sequence_size = averaged_ranges_size + 3;
	else
		state_sequence_size = averaged_ranges_size + 2;
				
	CHECK_EQ( state_sequence_size, blobData->shape(1) ) << "train net: supposed input sequence size check failed";
	
	if( multiclass ) {
		initializeSteer(steer_angles, yaw_resolution);
		CHECK_EQ(blobOut->shape(1), steer_angles.size());
	}

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

	goal_received = false;

	ros::Rate ctrl(control_rate);

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
		blobData->mutable_cpu_data()[range_data.size() + 1] = atan2( y_rel , x_rel );

		if( steer_feedback ) {

			if ( multiclass ) 
				blobData->mutable_cpu_data()[range_data.size() + 1] = prev_closest_steer;
			else
				blobData->mutable_cpu_data()[range_data.size() + 1] = prev_yaw_ref;
	
		}

//		for(int i = 0; i < state_sequence_size; i++) {
//			cout << blobData->mutable_cpu_data()[i] << "  ";
//		}

		online_net->Forward();
	
		int steer_label;
		float closest_steer = 0, yaw_ref = 0;

		if( multiclass ) {
			steer_label = blobArgmax->mutable_cpu_data()[0];
			closest_steer = steer_angles[steer_label];
			LOG_IF(INFO) << "Out index: " <<  steer_label
						     << " closest: " << closest_steer
							<< " measured: " << yaw_measured; 
			if( saturate(max_rotate_vel, -max_rotate_vel , closest_steer) ) {
				LOG(INFO) << "Angular velocity command saturating";
			}

		}
		else {
			yaw_ref = blobOut->mutable_cpu_data()[0];
			LOG_IF(INFO, !multiclass) << " Ref yaw: " << yaw_ref
							      << " measured: " << yaw_measured;

			if( saturate(max_rotate_vel, -max_rotate_vel, yaw_out) ) {
				LOG(INFO) << "Angular velocity command saturating";
			}
		
		}
		
		geometry_msgs::Twist cmd_out;

		bool got_command;

		if ( multiclass )
			got_command = ComputeNewVelocities(closest_steer, yaw_measured, 
										geometry_msgs::Twist drive_cmds);
		else
			got_command = ComputeNewVelocities(yaw_ref, yaw_measured, 
										geometry_msgs::Twist drive_cmds);

		if( got_command ) {
	
			LOG(INFO) << "Vel commands: Trasl: " << drive_cmds.linear.x
					<< " Rot: " << drive_cmds.angular.z;
			vel_command_pub_.publish(cmd_out);
	
		}
		else {
			LOG(ERROR) << "No command found for this control cycle!!";
		}

		online_step++;

		prev_ref_angular_z = angular;
		prev_ref_linear_x = linear;
		prev_meas_angular_z = meas_angular_z; 
		prev_meas_linear_x = meas_linear_x; 	

		prev_yaw_measured = yaw_measured;
		prev_yaw_ref = yaw_ref;
		prev_closest_steer = closest_steer;

      }

	 if ( point_distance(current_source, current_target) <= target_tolerance ) {
		 // current pos has achieved target within the tolerance
//		LOG(INFO) << "target reached";
		PublishZeroVelocity();
		goal_received = false;
	 }

	 prev_source = current_source;

	 ctrl.sleep();
	
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


/* yaw angle from the net is the reference value 
 * velocity commands selected are the ones getting closer
 * to the refrence in a control loop cycle time
 */

bool OnlineDeploy::ComputeNewVelocity(float yaw_ref, float yaw_measured, 
								geometry_msgs::Twist& drive_cmds) { 

	float min_rad_distance = M_PI_2;
	float closest_rot_vel = 0;
	float control_dt = 1 / control_rate;

	for( float vth_i = min_rotational_vel;
		vth_i < max_rotational_vel; vth_i += rotate_vel_res) {

		yaw_i = ComputeNewTheta(yaw_measured, vth_i, control_dt);
		float dist_i = fabs( yaw_ref - yaw_i );
		
		if ( dist_i < min_rad_distance ) {
			min_rad_distance = dist_i;
			closest_rot_vel = vth_i;
		}

	}

	if( min_rad_distance == M_PI_2 )
		return false;

	
	drive_cmds.angular.z = closest_rot_vel;

	float yaw_step = yaw_measured - yaw_ref;

	if( yaw_step > CRUISE_ANGLE )
		drive_cmds.linear.x = min_cruise_vel;
	else
		drive_cmds.linear.x = cruise_vel;

	
	return true;


}

} // namespace neural_network_planner





