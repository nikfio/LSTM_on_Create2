
#include <ros/ros.h>

#include <math.h>
#include <neural_network_planner/build_database.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <ctime>
#include <cmath>

#include <boost/lexical_cast.hpp>
#include <iostream>

#include "boost/scoped_ptr.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/db.hpp"
#include "caffe/util/io.hpp"
#include "caffe/util/rng.hpp"
#include "caffe/util/format.hpp"

#include <cstdio>
#include <cstdlib>


using namespace sensor_msgs;
using namespace nav_msgs;
using namespace move_base_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace visualization_msgs;	


using boost::lexical_cast;
using boost::scoped_ptr;
using std::cout;
using std::endl;


namespace neural_network_planner {


BuildDatabase::BuildDatabase(std::string& database_name) : private_nh("~")
{


	private_nh.param("set_size", set_size, 10000);
	private_nh.param("batch_size", batch_size, 100);	
	private_nh.param("scan_topic", scan_topic, std::string("/base_scan") );
	private_nh.param("goal_topic", goal_topic, std::string("/move_base/goal") );	
	private_nh.param("odom_topic", odom_topic, std::string("/odom") );
	private_nh.param("command_topic", command_topic, std::string("/cmd_vel") );
	private_nh.param("base_path", base_path, std::string("") );
	private_nh.param("averaged_ranges_size", averaged_ranges_size, 15 );
	private_nh.param("database_backend", backend, std::string("leveldb"));
	private_nh.param("logs_path", logs_path, std::string(""));
	private_nh.param<float>("minimal_step_distance", minimal_step_dist, 0.5); 
	private_nh.param<float>("sampling_rate", sampling_rate, 1);
	private_nh.param<float>("pos_update_threshold", pos_update_threshold, 0.001);
	private_nh.param("show_lines", show_lines, false);
	private_nh.param("crowdy", crowdy, false); 
	private_nh.param<float>("target_tolerance", target_tolerance, 0.08);
	private_nh.param("saturate_vel", saturate_vel, true);
	private_nh.param("command_tail", command_tail, true);

	FLAGS_log_dir = logs_path;
	FLAGS_alsologtostderr = 1;
	FLAGS_minloglevel = 0;

	prev_target = std::pair<float, float>(0,0);
	current_source = std::pair<float,float>(0,0);
	prev_source = std::pair<float,float>(0,0);
	
	range_data = vector<float>(averaged_ranges_size, 0);

	prev_ref_linear_x = prev_ref_angular_z = 0;	
	prev_meas_linear_x = prev_meas_angular_z = 0;

	// related neural network input size selected for this build_database run 
	state_sequence_size = averaged_ranges_size + 2;

	int labels_size = 2;

	timestep = database_counter = 0;
	db_writestep = 0;

	CHECK_EQ(set_size % batch_size, 0) << "set_size must be multiple of batch_size!";

	ros::NodeHandle db_nh("build_db");
	laserscan_sub_.subscribe(db_nh, scan_topic, 25);
	odom_sub_.subscribe(db_nh, odom_topic, 25);

	typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> StepPolicy;
	
	Synchronizer<StepPolicy> step_sync( StepPolicy(10), laserscan_sub_, odom_sub_);
	step_sync.registerCallback(boost::bind(&neural_network_planner::BuildDatabase::build_callback, this, _1, _2));

	ros::NodeHandle nh;
	goal_sub_ = nh.subscribe<MoveBaseActionGoal>(goal_topic , 1, boost::bind(&BuildDatabase::updateTarget_callback, this, _1));

	command_sub_ = nh.subscribe<geometry_msgs::Twist>(command_topic , 1, boost::bind(&BuildDatabase::updateCmdVel_callback, this, _1));
	

	net_ranges_pub_ = nh.advertise<LaserScan>("state_ranges", 1);
	if( show_lines ) {
		marker_pub_ = nh.advertise<Marker>("range_lines", 1);
	}

	time_t init = time(0);
	tm *init_tm = localtime(&init);	

	// states database with ref command tail initialization
	std::string states_ref_db_path = base_path + "states_reftail_db-" + lexical_cast<std::string>(init_tm->tm_mon+1) 
				              + "-" + lexical_cast<std::string>(init_tm->tm_mday) + "-" + lexical_cast<std::string>(init_tm->tm_hour) 
				              + "-" + lexical_cast<std::string>(init_tm->tm_min) + "_" + backend;
	scoped_ptr<caffe::db::DB> states_ref_database(caffe::db::GetDB(backend));
	states_ref_database->Open(states_ref_db_path, caffe::db::NEW);
	scoped_ptr<caffe::db::Transaction> states_ref_txn(states_ref_database->NewTransaction());

	// states database with measured command tail initialization
	std::string states_meas_db_path = base_path + "states_meastail_db-" + lexical_cast<std::string>(init_tm->tm_mon+1) 
				              + "-" + lexical_cast<std::string>(init_tm->tm_mday) + "-" + lexical_cast<std::string>(init_tm->tm_hour) 
				              + "-" + lexical_cast<std::string>(init_tm->tm_min) + "_" + backend;
	scoped_ptr<caffe::db::DB> states_meas_database(caffe::db::GetDB(backend));
	states_meas_database->Open(states_meas_db_path, caffe::db::NEW);
	scoped_ptr<caffe::db::Transaction> states_meas_txn(states_meas_database->NewTransaction());

	// labels measured signals database initialization
	std::string labels_meas_db_path = base_path + "labels_meas_db-" + lexical_cast<std::string>(init_tm->tm_mon+1) 
				              + "-" + lexical_cast<std::string>(init_tm->tm_mday) + "-" + lexical_cast<std::string>(init_tm->tm_hour) 
				              + "-" + lexical_cast<std::string>(init_tm->tm_min) + "_twist-variant_" + backend;
	scoped_ptr<caffe::db::DB> labels_meas_database(caffe::db::GetDB(backend));
	labels_meas_database->Open(labels_meas_db_path, caffe::db::NEW);
	scoped_ptr<caffe::db::Transaction> labels_meas_txn(labels_meas_database->NewTransaction());

	// labels reference signals database initialization
	std::string labels_ref_db_path = base_path + "labels_ref_db-" + lexical_cast<std::string>(init_tm->tm_mon+1) 
				              + "-" + lexical_cast<std::string>(init_tm->tm_mday) + "-" + lexical_cast<std::string>(init_tm->tm_hour) 
				              + "-" + lexical_cast<std::string>(init_tm->tm_min) + "_twist-variant_" + backend;
	scoped_ptr<caffe::db::DB> labels_ref_database(caffe::db::GetDB(backend));
	labels_ref_database->Open(labels_ref_db_path, caffe::db::NEW);
	scoped_ptr<caffe::db::Transaction> labels_ref_txn(labels_ref_database->NewTransaction());


	// creating a text file to debug database building - just first times
     // to check everything is right

	std::string check_text = base_path + "check_db-" + lexical_cast<std::string>(init_tm->tm_mon+1) 
				         + "-" + lexical_cast<std::string>(init_tm->tm_mday) + "-" + lexical_cast<std::string>(init_tm->tm_hour) 
				         + "-" + lexical_cast<std::string>(init_tm->tm_min) + "_" + backend;

	int print_check;
	FILE * table = fopen(check_text.c_str(), "w");
	if( table == NULL ) {
		cout << "Plot file opening failed.\n";
		exit(1);
	}

	bool actual_start = false;
	goal_received = false;

	ros::Rate store_rate(sampling_rate);

	while( ros::ok() && db_writestep < set_size) {

//	  LOG(INFO) << "Condition seq: " << crowdy << "  " << meas_linear_x << "  " 
//			  << meas_angular_z << "  " << goal_received << endl;

	  if(  ( fabs(meas_linear_x) > noise_level || fabs(meas_angular_z) > noise_level 
			||  crowdy ) && goal_received ) { 

		float x_rel = current_target.first - current_source.first;
		float y_rel = current_target.second - current_source.second;	 

		float distance = hypot( x_rel, y_rel);
		float relative_angle = fabs(atan2( y_rel , x_rel ) - current_orientation);
		
		
		/* STATE with reference command tail */

		std::string state_ref_value;
		caffe::Datum state_ref_datum;
		state_ref_datum.set_channels(state_sequence_size);
		state_ref_datum.set_height(1);
		state_ref_datum.set_width(1);
		
		// storing the state data 				
		for(int i = 0; i < range_data.size(); i++) {
			state_ref_datum.add_float_data(range_data[i]);
		}		

		state_ref_datum.add_float_data(distance);
		state_ref_datum.add_float_data(relative_angle);

		if( command_tail ) {
			state_ref_datum.add_float_data(prev_ref_linear_x);
			state_ref_datum.add_float_data(prev_ref_angular_z);
		}

		state_ref_datum.set_encoded(false);
		state_ref_datum.SerializeToString(&state_ref_value);	
		std::string key_str = caffe::format_int(db_writestep, 8);
		states_ref_txn->Put(key_str, state_ref_value);

		std::string state_meas_value;
		caffe::Datum state_meas_datum;
		state_meas_datum.set_channels(state_sequence_size);
		state_meas_datum.set_height(1);
		state_meas_datum.set_width(1);
		
		// storing the state data 				
		for(int i = 0; i < range_data.size(); i++) {
			state_meas_datum.add_float_data(range_data[i]);
		}		
		
		state_meas_datum.add_float_data(distance);
		state_meas_datum.add_float_data(relative_angle);

		if( command_tail ) {
			state_meas_datum.add_float_data(prev_meas_linear_x);
			state_meas_datum.add_float_data(prev_meas_angular_z);
		}
		
		state_meas_datum.set_encoded(false);
		state_meas_datum.SerializeToString(&state_meas_value);	
		states_meas_txn->Put(key_str, state_meas_value);

		/* LABELS with reference command */

		std::string label_ref_value;
		caffe::Datum label_ref_datum;
		label_ref_datum.set_channels(labels_size);
		label_ref_datum.set_height(1);	
		label_ref_datum.set_width(1);

		// order of loading labels is unique and decided here    
		label_ref_datum.add_float_data(ref_linear_x);
		label_ref_datum.add_float_data(ref_angular_z);
		label_ref_datum.set_encoded(false);
		
		// using same key as state one - consistent accessing to databases
		label_ref_datum.SerializeToString(&label_ref_value);
		labels_ref_txn->Put(key_str, label_ref_value);

		/* LABELS with measured command */

		std::string label_meas_value;
		caffe::Datum label_meas_datum;
		label_meas_datum.set_channels(labels_size);
		label_meas_datum.set_height(1);	
		label_meas_datum.set_width(1);

		// order of loading labels is unique and decided here    
		label_meas_datum.add_float_data(meas_linear_x);
		label_meas_datum.add_float_data(meas_angular_z);
		label_meas_datum.set_encoded(false);
		
		// using same key as state one - consistent accessing to databases
		label_meas_datum.SerializeToString(&label_meas_value);
		labels_meas_txn->Put(key_str, label_meas_value);

		table = fopen(check_text.c_str(), "a");
		if( table == NULL ) {
			cout << "Plot file opening failed.\n";
			exit(1);
		}
		for(int i = 0; i < range_data.size(); i++) {
		 fprintf(table, "%.4f   ", range_data[i]); 
		}
		 fprintf(table, "%.4f   ", distance); 
		 fprintf(table, "%.4f \n  ", relative_angle); 
		 fprintf(table, "DB STEP %d   LABELS   %.4f  %.4f    %.4f  %.4f \n ", 
					db_writestep, ref_linear_x, ref_angular_z, meas_linear_x, meas_angular_z); 
		 fclose(table);
		

		if( ++db_writestep % batch_size == 0 ) { // commit the batch to dbs

			states_ref_txn->Commit();	
			states_meas_txn->Commit();				
			labels_ref_txn->Commit();
			labels_meas_txn->Commit();
			states_ref_txn.reset(states_ref_database->NewTransaction());
			states_meas_txn.reset(states_meas_database->NewTransaction());			
			labels_ref_txn.reset(labels_ref_database->NewTransaction());			
			labels_meas_txn.reset(labels_meas_database->NewTransaction());

      	} 

		LOG(INFO) << "Stored step  " << db_writestep;

		// update the tails
		prev_ref_linear_x = ref_linear_x;
		prev_ref_angular_z = ref_angular_z;
		prev_meas_linear_x = meas_linear_x;
		prev_meas_angular_z = meas_angular_z;

	     
	  } // check available step
	  


	  if ( point_distance(current_source, current_target) <= target_tolerance ) {
		 // current pos has achieved target within tolerance
		LOG(INFO) << "target reached";
		goal_received = false;
	  }

	  prev_source = current_source;

	  store_rate.sleep();
	
	  ros::spinOnce();

	}

	
	if( db_writestep % batch_size != 0) { // Last commit if needed for a safe closing

		LOG(INFO) << "Closing DATABASES ";
		states_ref_txn->Commit();	
		states_meas_txn->Commit();		
		labels_ref_txn->Commit();
		labels_meas_txn->Commit();
	}

	
	LOG(INFO) << "In databases " << states_ref_db_path << " have been stored " << db_writestep << " steps";


	
}

BuildDatabase::~BuildDatabase() {


}

	

/* this thread is collecting navigation data - first variant - labels are set depending
 * on the real path effectively taken by the robot - output of controller (local planner)
 */
void BuildDatabase::build_callback(const LaserScan::ConstPtr& laser_msg, 
							const Odometry::ConstPtr& odom_msg)
{
	
	timestep++;
//	ROS_INFO("Database_callback timestep: %d", timestep);
	


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

//	LOG(INFO) << "Range measures total num:" << range_num;
//     LOG(INFO) << "Scan message ranges size: " << ranges.size();
//	LOG(INFO) << "ANGLE INCREMENT: " << laser_msg->angle_increment;
	
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

void BuildDatabase::updateTarget_callback( const MoveBaseActionGoal::ConstPtr& actiongoal_msg) {

	LOG(INFO) << "Updating target pose";
	current_target.first = actiongoal_msg->goal.target_pose.pose.position.x;
	current_target.second = actiongoal_msg->goal.target_pose.pose.position.y;
	goal_received = true;

}

void BuildDatabase::updateCmdVel_callback( const geometry_msgs::Twist::ConstPtr& cmdvel_msg ) {

	cmdvel_time = ros::Time::now();
//	LOG(INFO) << "Updating ref commands from navigation stack";
	ref_linear_x = cmdvel_msg->linear.x;
	ref_angular_z = cmdvel_msg->angular.z;

}

float BuildDatabase::Step_dist() {

	return hypot(current_source.second - prev_source.second, current_source.first - prev_source.first);

}

float point_distance(std::pair<float, float>& start_point, std::pair<float, float>& end_point) 
{

	return hypot(end_point.second - start_point.second, end_point.first - start_point.first);

}

int  saturate(float neg_lim, float pos_lim, float& value) 
 {

	if( value < neg_lim ) { 
		value = neg_lim;
		return 1;
	}
	else if( value > pos_lim ) {
		value = pos_lim;
	  	return 2;
	}
	else 
		return 0;

 }


} // namespace neural_network_planner


