

// ROS related
#include <neural_network_planner/train_validate.h>


// caffe related
#include "caffe/layers/data_layer.hpp"
#include "glog/logging.h"

#include <boost/lexical_cast.hpp>

#include <ctime>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <cstdlib>

using std::cout;
using std::endl;
using std::cin;
using std::string;

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace move_base_msgs;
using namespace message_filters; 
using namespace geometry_msgs; 

using boost::lexical_cast;

namespace neural_network_planner {

	TrainValidateRNN::TrainValidateRNN(string& process_name) : private_nh("~")
	{
		
		private_nh.param("resume", resume, false);
		private_nh.param("solver_config", solver_conf, std::string(""));
		private_nh.param("trained_weights", trained, std::string("") );
		private_nh.param("GPU", GPU, true );
		private_nh.param("total_iterations", total_iterations, 5000); 
		private_nh.param("iter_size", iter_size, 1 );
		private_nh.param("batch_updates", batch_updates, 1 );	
		private_nh.param("validation_frequency", val_freq, 4 );			
		private_nh.param("time_sequence", time_sequence, 10);
		private_nh.param("folder_path", folder_path, std::string(""));
		private_nh.param("averaged_ranges_size", averaged_ranges_size, 10 );
		private_nh.param("validation_frequency", val_freq, 2 );
		private_nh.param("logs_path", logs_path, std::string(""));
		private_nh.param("steer_feedback", steer_feedback, true);
		private_nh.param("multiclass", multiclass, true);
		private_nh.param("steer_resolution", steer_resolution, 10); 
		private_nh.param("online_training", online_training, true); 
		private_nh.param("scan_topic", scan_topic, std::string("/base_scan") );
		private_nh.param("goal_topic", goal_topic, std::string("/move_base/goal") );	
		private_nh.param("odom_topic", odom_topic, std::string("/odom") );
		private_nh.param<float>("pos_update_threshold", pos_update_threshold, 0.001);
		private_nh.param<float>("target_tolerance", target_tolerance, 0.1);
		private_nh.param<float>("step_resolution", step_resolution, 0.10); 
		private_nh.param("dist_scale", dist_scale, 10); 
		private_nh.param("yaw_scale", yaw_scale, 10); 
		private_nh.param("checking_rate", checking_rate, 1);
		private_nh.param("max_steer_angle", max_steer_angle, 90);
		private_nh.param("min_steer_angle", min_steer_angle, -90);


		FLAGS_log_dir = logs_path;
		FLAGS_alsologtostderr = 1;
		FLAGS_minloglevel = 0;

		prev_target = std::pair<float, float>(0,0);
		current_source = std::pair<float,float>(0,0);
		prev_source = std::pair<float,float>(0,0);
	
		range_data = vector<float>(averaged_ranges_size, 0);

		yaw_measured = prev_yaw_measured = 0;

		if (GPU) {
	    		caffe::Caffe::set_mode(caffe::Caffe::GPU);
			LOG(INFO) << "GPU mode"; 
	  	 } else {
	    		caffe::Caffe::set_mode(caffe::Caffe::CPU);
			LOG(INFO) << "CPU mode";
	  	}

		
		// initializing the neural network - parsing the solver config file
		FLAGS_minloglevel = 1;
		LOG(WARNING) << "Parsing solver config " << solver_conf;
		caffe::SolverParameter solver_param;
		caffe::ReadProtoFromTextFileOrDie(solver_conf, &solver_param);
		solver.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param));	

		FLAGS_minloglevel = 0;

		net = solver->net();

		// basic checking for minimal functioning
		CHECK(net->has_blob("data"));	
		CHECK(net->has_blob("label"));	
		CHECK(net->has_blob("loss"));
		CHECK(net->has_blob("out"));

		if( multiclass ) {
			CHECK(net->has_blob("argmax"));
			CHECK(net->has_blob("accuracy"));
		}	

		blobData = net->blob_by_name("data");
		blobClip = net->blob_by_name("clip");
		blobLabel = net->blob_by_name("label");
		blobLoss = net->blob_by_name("loss");
		blobOut  = net->blob_by_name("out");
		blobAccu = net->blob_by_name("accuracy");	
		blobArgmax  = net->blob_by_name("argmax");
		blobSoftmax  = net->blob_by_name("softmax");				
		
		test_net = solver->test_nets()[0];

		// basic checking for minimal functioning
		CHECK(test_net->has_blob("data"));	
		CHECK(test_net->has_blob("label"));	
		CHECK(test_net->has_blob("out"));	
		if( multiclass ) {
			CHECK(test_net->has_blob("argmax"));	
			CHECK(test_net->has_blob("accuracy"));
		}
	
		test_blobData = test_net->blob_by_name("data");
		test_blobClip = test_net->blob_by_name("clip");
		test_blobLabel = test_net->blob_by_name("label");
		test_blobLoss = test_net->blob_by_name("loss");
		test_blobOut  = test_net->blob_by_name("out");
		test_blobAccu = net->blob_by_name("accuracy");
		test_blobArgmax  = test_net->blob_by_name("argmax");	
		test_blobSoftmax  = test_net->blob_by_name("softmax");			

		
		// input state size checks
		if( steer_feedback ) 
			state_sequence_size = averaged_ranges_size + 4;
		else
			state_sequence_size = averaged_ranges_size + 3;
					
		CHECK_EQ( state_sequence_size, blobData->shape(1) ) << "train net: supposed input sequence size check failed";
		CHECK_EQ( state_sequence_size, test_blobData->shape(1) ) << "validate net: supposed input sequence size check failed";

		CHECK_EQ( test_blobData->shape(0), blobData->shape(0) ) << "minibatch mismatch";

		minibatch = blobData->shape(0);

		FLAGS_minloglevel = 0;

		if( multiclass ) {
			initializeSteer(steer_angles, steer_resolution, min_steer_angle, max_steer_angle);
			CHECK_EQ(blobOut->shape(1), steer_angles.size());
		}
		else
			CHECK_EQ(blobOut->shape(1), blobLabel->shape(1));
		
		
		solver_param.set_iter_size(iter_size);
	
		FLAGS_minloglevel = 0;
	
		if( resume ) { // resume from pre-trained weights
			
			LOG(INFO) << "Selected resume training from: " << trained;
			net->CopyTrainedLayersFrom(trained);

		}
		else {
			LOG(INFO) << "Selected start a new training";
		}

		time_t now = time(0);
		tm *local = localtime(&now);

		const string prefix = folder_path + net->name() + "_" + lexical_cast<string>(local->tm_mon+1) 
				            + "-" + lexical_cast<string>(local->tm_mday) + "-" + lexical_cast<string>(local->tm_hour);

		solver_param.set_snapshot_prefix(prefix);

		LOG(INFO) << "Net loaded: " << net->name();
		LOG(INFO) << "Minibatch size: " << minibatch;
		LOG(INFO) << "Forward - backward per batch (gradients accumulated): " << iter_size;	
		LOG(INFO) << "Number of updates per batch: " << batch_updates;
	
		string matlab_plot = folder_path + "Test_" + net->name() + "_" + lexical_cast<string>(local->tm_mon+1) 
				   		  + "-" + lexical_cast<string>(local->tm_mday) + "-" + lexical_cast<string>(local->tm_hour) 
				   		  + "-" + lexical_cast<string>(local->tm_min);
				
		int print_check;
		FILE * plot = fopen(matlab_plot.c_str(), "w");
		if( plot == NULL ) {
			cout << "Plot file opening failed.\n";
			exit(1);
		}

		ros::NodeHandle db_nh("build_db");
		laserscan_sub_.subscribe(db_nh, scan_topic, 25);
		odom_sub_.subscribe(db_nh, odom_topic, 25);

		typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> StepPolicy;
		
		Synchronizer<StepPolicy> step_sync( StepPolicy(10), laserscan_sub_, odom_sub_);
		step_sync.registerCallback(boost::bind(&neural_network_planner::TrainValidateRNN::build_callback, this, _1, _2));
	
		ros::NodeHandle nh;
		goal_sub_ = nh.subscribe<MoveBaseActionGoal>(goal_topic, 1, boost::bind(&TrainValidateRNN::updateTarget_callback, this, _1));

		net_ranges_pub_ = nh.advertise<LaserScan>("state_ranges", 1);
		

		print_check = fprintf(plot, "TEST: %s DATE: %d %d %d:%d \n" 
				    " PARAMETERS:  \n"
				    " minibatch = %d \n iter_size = %d \n batch_updates = %d \n "
				    " base_learning_rate = %.5f \n"
				    " weight_decay = %f \n "
				    " input_size = %d \n "
				    " steer_feedback = %d \n "
				    " loss_data = [ \n ",
				     net->name().c_str(), local->tm_mon+1, 
				     local->tm_mday, local->tm_hour, local->tm_min,
				     minibatch, iter_size, batch_updates, 
				     solver->param().base_lr(),
				     solver->param().weight_decay(),
					state_sequence_size,
					steer_feedback );
	
		if(print_check <= 0) {
		   printf("File: writetofile() failed\n");
		   exit(1);
		}
		fclose(plot);

		/* populate the clip blobs
           * by chosing a constant time sequence here in this implementation
           * clip blobs values are always the same 
           * this operation is done only here one time for all
           */

		// populate training clip blob 
		for(int i = 0; i < minibatch; i++) {

			if( i % state_sequence_size == 0 ) {
				blobClip->mutable_cpu_data()[i] = 0;
			}
			else {
				blobClip->mutable_cpu_data()[i] = 1;
			}
		
		}
	
		// populate validating clip blob 
		for(int i = 0; i < minibatch; i++) {

			if( i % state_sequence_size == 0 ) {
				test_blobClip->mutable_cpu_data()[i] = 0;
			}
			else {
				test_blobClip->mutable_cpu_data()[i] = 1;
			}

		}
	
		FLAGS_minloglevel = 0;

		int SHOW_ITER_LOG = 1;
		int validation_test = 0;

		int data_index = 0;

		ros::Rate rate(checking_rate);
			
		LOG(INFO) << "TRAIN ONLINE PROCESS INITIALIZED";

		int iter = 1;

		while( ros::ok() && iter < total_iterations ) { //  begin training process

			TRAIN = iter % val_freq == 0 ? false : true;

			if(  ( fabs(meas_linear_x) > noise_level || fabs(meas_angular_z) > noise_level )
			 		&& goal_received ) { 

				blobData_ = TRAIN ? blobData : test_blobData;
				blobLabel_ = TRAIN ? blobLabel : test_blobLabel;
		
				float x_rel = current_target.first - current_source.first;
				float y_rel = current_target.second - current_source.second;	 

				float closest_steer = 0;		

				if ( data_index > 0 && data_index < (minibatch + 1) ) {
					
					if ( multiclass ) {
						
						int steer_label = getClosestSteer( steer_angles, yaw_measured - prev_yaw_measured, closest_steer);
						blobLabel_->mutable_cpu_data()[data_index - 1] = steer_label;
									 
//						printf("Label math: prev_measured: %.4f measured:  %.4f closest:  %.4f index : %d  \n", 
//								prev_yaw_measured, yaw_measured, closest_steer, steer_label);
		
					}
					else {

						blobLabel_->mutable_cpu_data()[data_index - 1] =
											yaw_measured - prev_yaw_measured;

//						printf("Label math: prev_measured: %.4f measured:  %.4f \n ", 
//									prev_yaw_measured, yaw_measured );
 
					}

				}			

				

				if(data_index < minibatch ) {

				int base_position = data_index * state_sequence_size;


				for(int i = 0; i < range_data.size(); i++) {
					blobData_->mutable_cpu_data()[base_position + i] = range_data[i];
				}		

				blobData_->mutable_cpu_data()[base_position + range_data.size()] = hypot( x_rel, y_rel) * dist_scale;
				blobData_->mutable_cpu_data()[base_position + range_data.size() + 1] = atan2( y_rel , x_rel );
				blobData_->mutable_cpu_data()[base_position + range_data.size() + 2] = yaw_measured * yaw_scale;
 		
				if( steer_feedback ) {

					if( multiclass ) {
						blobData_->mutable_cpu_data()[base_position + range_data.size() + 3] = prev_closest_steer;
									
					}
					else {
						blobData_->mutable_cpu_data()[base_position + range_data.size() + 3] = 
														yaw_measured - prev_yaw_measured;
				
					}
				
				}
			
//				cout << "INPUT DATA: ";
//				for( int i = 0; i < state_sequence_size; i++) {
//					cout << blobData_->cpu_data()[base_position + i] << "  ";
//				}
//				cout << endl;


				}

				data_index++;
				prev_yaw_measured = yaw_measured;
				prev_source = current_source;
				prev_closest_steer = closest_steer;

			}
		

			if ( point_distance(current_source, current_target) <= target_tolerance ) {
		 		// current pos has achieved target within tolerance
//				LOG(INFO) << "target reached";
				goal_received = false;
		     }

			
			
			if( data_index == (minibatch + 1) && TRAIN ) {	// begin train iteration 

			float Train_loss = 0.0f;	
			float Train_accu = 0.0f;
		
			solver->Step(batch_updates);

			Train_loss = blobLoss->mutable_cpu_data()[0];

			if( multiclass )
				Train_accu = blobAccu->mutable_cpu_data()[0];

//			char answer;
//			cout << "TRAINING: Want to check batch output? (y/n)" << endl;
//			cin >> answer;
//			if ( answer == 'y' ) {

//				for(int i = 0; i < minibatch; i++) {  

//					printf("Batch %d  sample %d  State input sequence: ", iter, i );
//					for(int j = 0; j < state_sequence_size; j++) {
//						printf(" %.4f  ",  blobData->mutable_cpu_data()[i * state_sequence_size + j]);
//					}
//					cout << endl;

//					printf("Batch %d  sample %d  NET OUTS: ", iter, i);   


//					for(int l=0; l < blobOut->channels(); l++) { 
//		
//						if(multiclass) {
//							printf(" %.4f   ", blobSoftmax->mutable_cpu_data()[i * blobOut->channels() + l]);
//						}	
//						else 
//							printf(" %.4f   ", blobOut->mutable_cpu_data()[i * blobOut->channels() + l]);
//					}
//			
//					cout << endl;
//					if( multiclass )
//						printf("ARGMAX: %.3f \n", blobArgmax->cpu_data()[0]);

//					printf("Batch %d  sample %d  LABELS OUTS: ", iter, i); 
//					for(int l=0; l < blobLabel->channels(); l++) {  
//		
//						printf(" %.4f  ", blobLabel->mutable_cpu_data()[i * blobLabel->channels() + l]);
//			
//					}
//					cout << endl;

//					printf("Batch %d  sample %d  Loss: %.5f \n", iter, i, blobLoss->mutable_cpu_data()[0] );
//		
//					cout << "Wanna pass forward? (y/n)" << endl;
//					cin >> answer;

//					if(answer == 'y')
//						break;

//				}
//					

//			}

//			printf("Batch %d  LABELS OUTS: ", iter); 
//			for(int i = 0; i < minibatch; i++) {
//				
//				for(int l=0; l < blobLabel->channels(); l++) { 
//						printf(" %.3f  ", blobLabel->mutable_cpu_data()[i * blobLabel->channels() + l]);
//				}
//			}
//		
//			cout << endl;

			if ( iter % SHOW_ITER_LOG == 0 ) {
				LOG(WARNING) << "TRAIN ITERATION: " << iter 
	 				        << " AVERAGE LOSS: " << Train_loss	
						   << "  AVERAGE ACCURACY: "  << Train_accu;
			}
		
			plot = fopen(matlab_plot.c_str(), "a");
			if( plot == NULL ) {
				cout << "Plot file opening failed.\n";
				exit(1);
			}
			fprintf(plot, "    %.4f   %.4f  ", Train_loss, Train_accu); 
			fclose(plot);			
		
			// end train iteration

			iter++;
			data_index = 0;

			} 
			else if( data_index == (minibatch + 1) && !TRAIN ) { // begin validation iteration
		
				validation_test++;
			
				float Test_loss = 0.0f;
				float Test_accu = 0.0f;	

				UpdateTestNet();
			
				test_net->Forward();	
		
				Test_loss = test_blobLoss->mutable_cpu_data()[0];	
				if( multiclass )
					Test_accu += blobAccu->mutable_cpu_data()[0];		

//				char answer;
//				cout << "VALIDATING: Want to check batch output? (y/n)" << endl;
//				cin >> answer;
//				if ( answer == 'y' ) {

//					for(int i = 0; i < minibatch; i++) { 
// 
//						printf("Batch %d  sample %d  State input sequence: ", iter, i );
//						for(int j = 0; j < state_sequence_size; j++) {
//							printf(" %.4f  ",  test_blobData->mutable_cpu_data()[i * state_sequence_size + j]);
//						}
//						getchar();
//						cout << endl;

//						printf("Batch %d  sample %d  NET OUTS: ", iter, i);   
//						for(int l=0; l < blobOut->channels(); l++) { 
//		
//							if(multiclass) {
//								printf(" %.4f   ", test_blobSoftmax->mutable_cpu_data()[i * blobOut->channels() + l]);
//							}	
//							else 
//								printf(" %.4f   ", test_blobOut->mutable_cpu_data()[i * blobOut->channels() + l]);
//			
//						}
//						cout << endl;
//						
//						if( multiclass )
//							printf("ARGMAX: %.3f \n", test_blobArgmax->mutable_cpu_data()[0]);

//						printf("Batch %d  sample %d  LABELS OUTS: ", iter, i); 
//						for(int l=0; l < blobLabel->channels(); l++) {  
//		
//							printf(" %.4f  ", test_blobLabel->mutable_cpu_data()[i * test_blobLabel->channels() + l]);
//			
//						}
//						cout << endl;

//						printf("Batch %d  sample %d   Test Loss: %.5f \n", iter, i, test_blobLoss->mutable_cpu_data()[0] );
//		
//						cout << "Wanna pass forward? (y/n)" << endl;
//						cin >> answer;

//						if(answer == 'y')
//							break;

//				     }
//					

//				}


				LOG(WARNING) << "TEST ITERATION : " << validation_test 
				        << "  AVERAGE LOSS: "  << Test_loss
					   << "  AVERAGE ACCURACY: "  << Test_accu;	

				plot = fopen(matlab_plot.c_str(), "a");
				if( plot == NULL ) {
					cout << "Plot file opening failed.\n";
					exit(1);
				}
				fprintf(plot, "    %.4f   %.4f ; \n", Test_loss, Test_accu); 
				fclose(plot);		
				
				// end validation test

				data_index = 0;
				iter++;				
											
			}  

	  rate.sleep();
	
	  ros::spinOnce();

	
	} // training process	
		
	} 


	TrainValidateRNN::~TrainValidateRNN() 
	{
		
	}

	void TrainValidateRNN::UpdateTestNet()
	{
		caffe::NetParameter net_param;
		net->ToProto(&net_param);
		net_param.mutable_state()->set_phase(caffe::TEST);
		test_net->CopyTrainedLayersFrom(net_param);	
	};

} // namespace neural_network_planner


