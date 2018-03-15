

// ROS related
#include <neural_network_planner/train_database.h>


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

const int train_check = 1000000;
const int validate_check = 10000000;

namespace neural_network_planner {

	TrainDatabase::TrainDatabase(string& process_name) : private_nh("~")
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
		private_nh.param("logs_path", logs_path, std::string(""));
		private_nh.param("steer_feedback", steer_feedback, true);
		private_nh.param("multiclass", multiclass, true);
		private_nh.param("train_set_size", train_set_size, 0 );
		private_nh.param("validate_set_size", validate_set_size, 0 );
		private_nh.param("epochs", epochs, 1000 ); 


		FLAGS_log_dir = logs_path;
		FLAGS_alsologtostderr = 1;
		FLAGS_minloglevel = 0;

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
		test_blobAccu = test_net->blob_by_name("accuracy");
		test_blobArgmax  = test_net->blob_by_name("argmax");	
		test_blobSoftmax  = test_net->blob_by_name("softmax");	
		test_lstm1 =  test_net->blob_by_name("fc1");	

		train_batch_size = blobData->shape(0);
		validate_batch_size = test_blobData->shape(0);

		train_batch_num = train_set_size / train_batch_size;
		validate_batch_num = validate_set_size / validate_batch_size;	

		
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


		print_check = fprintf(plot, "TEST: %s DATE: %d %d %d:%d \n" 
				    " PARAMETERS:  \n"
				    " minibatch = %d \n iter_size = %d \n batch_updates = %d \n "
				    " base_learning_rate = %.5f \n"
				    " weight_decay = %f \n "
				    " input_size = %d \n "
				    " output_size = %ld \n "
				    " steer_feedback = %d \n "
				    " loss_data = [ \n ",
				     net->name().c_str(), local->tm_mon+1, 
				     local->tm_mday, local->tm_hour, local->tm_min,
				     minibatch, iter_size, batch_updates, 
				     solver->param().base_lr(),
				     solver->param().weight_decay(),
					state_sequence_size,
					multiclass ? steer_angles.size() : 1,
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
		for(int i = 0; i < train_batch_size; i++) {

			if( i == 0 ) {
				for(int j = 0; j < state_sequence_size; j++) {
					blobClip->mutable_cpu_data()[i * state_sequence_size + j] = 0;
				}
			}
			else {
				for(int j = 0; j < state_sequence_size; j++) {
					blobClip->mutable_cpu_data()[i * state_sequence_size + j] = 1;
				}
			}
		
		}
	
		// populate validating clip blob 
		for(int i = 0; i < validate_batch_size; i++) {

			if( i == 0 ) {
				for(int j = 0; j < state_sequence_size; j++) {
					test_blobClip->mutable_cpu_data()[i * state_sequence_size + j] = 0;
				}
			}
			else {
				for(int j = 0; j < state_sequence_size; j++) {
					test_blobClip->mutable_cpu_data()[i * state_sequence_size + j] = 1;
				}
			}

		}
	
		FLAGS_minloglevel = 1;
			
		LOG(INFO) << "TRAIN DATABASE PROCESS INITIALIZED";

		int SHOW_EPOCH_LOG = 1;
		int validation_test = 0;
		int epoch = 1;		

		float Test_loss = 0.0f;
		float Test_accu = 0.0f;
		
		float Train_loss = 0.0f;	
		float Train_accu = 0.0f;	

		while( ros::ok() && epoch < epochs ) { // training process
		
			TRAIN = true;

			Train_loss = 0.0f;	
			Train_accu = 0.0f;	
		
			for(int k = 1; k <= train_batch_num; k++) { // training batch 

				solver->Step(batch_updates);

				Train_loss += blobLoss->cpu_data()[0];
				if( multiclass )
					Train_accu += blobAccu->cpu_data()[0];


				if( epoch % train_check == 0 ) {

				char answer;
				cout << "TRAINING: Want to check batch output? (y/n)" << endl;
				cin >> answer;
				if ( answer == 'y' ) {

				for(int i = 0; i < train_batch_size; i++) {  

					printf("Batch %d  sample %d  State input sequence: ", k, i );
					for(int j = 0; j < state_sequence_size; j++) {
						printf(" %.4f  ",  blobData->cpu_data()[i * state_sequence_size + j]);
					}

					cout << endl;

					printf("Batch %d  sample %d  NET OUTS: ", k, i);   
					for(int l=0; l < blobOut->channels(); l++) { 
		
						if(multiclass) {
							printf(" %.4f   ", blobSoftmax->cpu_data()[i * blobOut->channels() + l]);
						}	
						else 
							printf(" %.4f   ", blobOut->cpu_data()[i * blobOut->channels() + l]);
					}
			
					cout << endl;
					

					printf("Batch %d  sample %d  LABELS OUTS: ", k, i); 
					for(int l=0; l < blobLabel->channels(); l++) {  
		
						printf(" %.4f  ", blobLabel->cpu_data()[i * blobLabel->channels() + l]);
			
					}
					cout << endl;

					if( multiclass ) {
						printf("ARGMAX: %.3f \n", blobArgmax->cpu_data()[i]);
						printf("ACCURACY: %.4f   %.4f \n", blobAccu->cpu_data()[i], Train_accu);
						
					}

					printf("Batch %d  sample %d  Loss: %.5f \n", k, i, blobLoss->cpu_data()[0] );
		
					cout << "Wanna pass forward? (y/n)" << endl;
					cin >> answer;

					if(answer == 'y')
						break;

				}
					
				}

				}

			// end training batch
	
			} 

			Train_loss /= train_batch_num;
			Train_accu /= train_batch_num;

			epoch++;
 
			if ( SHOW_EPOCH_LOG ) {
				LOG(WARNING) << "TRAIN EPOCH: " << epoch 
	 				        << " AVERAGE LOSS: " << Train_loss
						   << " AVERAGE ACCURACY: " << Train_accu;
			}
		
			plot = fopen(matlab_plot.c_str(), "a");
			if( plot == NULL ) {
				cout << "Plot file opening failed.\n";
				exit(1);
			}
			fprintf(plot, " %.4f    %.4f   ",Train_accu,  Train_loss); 
			fclose(plot);
			
			if( epoch % val_freq == 0 ) { // begin validation test
		
				validation_test++;
			
				Test_loss = 0.0f;
				Test_accu = 0.0f;	

				UpdateTestNet();
				
				for(int k=1; k <= validate_batch_num; k++) { // validation batch 

					test_net->Forward();	
		
					Test_loss += test_blobLoss->cpu_data()[0]; 
					if( multiclass )
						Test_accu += test_blobAccu->cpu_data()[0];
					
					if( validation_test % validate_check == 0 ) {

					char answer;
					cout << "VALIDATING: Want to check batch output? (y/n)" << endl;
					cin >> answer;
					if ( answer == 'y' ) {

						for(int i = 0; i < validate_batch_size; i++) {  


						printf("Batch %d  sample %d  State input sequence: ", k, i );
						for(int j = 0; j < state_sequence_size; j++) {
							printf(" %.4f  ",  test_blobData->cpu_data()[i * state_sequence_size + j]);
						}
						getchar();
						cout << endl;

						printf("Batch %d  sample %d  NET OUTS: ", k, i);   
						for(int l=0; l < blobOut->channels(); l++) { 
		
							if(multiclass) {
								printf(" %.4f   ", test_blobSoftmax->cpu_data()[i * blobOut->channels() + l]);
							}	
							else 
								printf(" %.4f   ", test_blobOut->cpu_data()[i * blobOut->channels() + l]);
			
						}
						cout << endl;

						printf("Batch %d  sample %d  LABEL OUT: ", k, i); 
						printf(" %.4f  ", test_blobLabel->cpu_data()[i]);
			
						
						cout << endl;
	
						if( multiclass ) {
							printf("ARGMAX: %.3f \n", test_blobArgmax->cpu_data()[i]);
							printf("ACCURACY: %.4f   %.4f \n", test_blobAccu->cpu_data()[i], Test_accu);
							
						}

						printf("Batch %d  sample %d   Test Loss: %.5f \n", k, i, test_blobLoss->cpu_data()[0] );
		
						cout << "Wanna pass forward? (y/n)" << endl;
						cin >> answer;

						if(answer == 'y')
							break;

				     }
					
					}

					}
											
				}  // end validation batch

				
				Test_loss /= validate_batch_num;

				Test_accu /= validate_batch_num;
		
				LOG(WARNING) << "VALIDATION TEST: " << validation_test 
				        << "  AVERAGE LOSS: "  << Test_loss
					   << "  AVERAGE ACCURACY: "  << Test_accu;	

		

			 } // end validation test 

			 plot = fopen(matlab_plot.c_str(), "a");
			 if( plot == NULL ) {
			 	cout << "Plot file opening failed.\n";
				exit(1);
			 }
			 fprintf(plot, "    %.4f   %.4f ; \n", Test_loss, Test_accu); 
			 fclose(plot);		
				
		} // training process	
		
	} 


	TrainDatabase::~TrainDatabase() 
	{
		
	}

	void TrainDatabase::UpdateTestNet()
	{
		caffe::NetParameter net_param;
		net->ToProto(&net_param);
		net_param.mutable_state()->set_phase(caffe::TRAIN);
		test_net->CopyTrainedLayersFrom(net_param);	
	};

} // namespace neural_network_planner


