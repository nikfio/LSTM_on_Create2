

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

using boost::lexical_cast;


namespace neural_network_planner {

	TrainValidateRNN::TrainValidateRNN(string& process_name) : private_nh("~")
	{
		
		private_nh.param("resume", resume, false);
		private_nh.param("solver_config", solver_conf, std::string(""));
		private_nh.param("trained_weights", trained, std::string("") );
		private_nh.param("GPU", GPU, true );
		private_nh.param("iter_size", iter_size, 1 );
		private_nh.param("batch_updates", batch_updates, 1 );		
		private_nh.param("train_set_size", train_set_size, 0 );
		private_nh.param("validate_set_size", validate_set_size, 0 );
		private_nh.param("epochs", epochs, 1000 );				
		private_nh.param("time_sequence", time_sequence, 10);
		private_nh.param("folder_path", folder_path, std::string(""));
		private_nh.param("averaged_ranges_size", averaged_ranges_size, 22 );
		private_nh.param("validation_test_frequency", val_freq, 2 );
		private_nh.param("logs_path", logs_path, std::string(""));
		private_nh.param("command_feedback", command_feedback, true);
		private_nh.param("feedback_type", tail_type, std::string(""));
		private_nh.param("output_size", out_size, 1); 

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

		FLAGS_minloglevel = 0;

		net = solver->net();

		// basic checking for minimal functioning
		CHECK(net->has_blob("data"));	
		CHECK(net->has_blob("labels"));	
		CHECK(net->has_blob("loss"));
		CHECK(net->has_blob("out"));
		CHECK(net->has_layer("data"));		

		blobData = net->blob_by_name("data");
		blobClip = net->blob_by_name("clip");
		blobLabel = net->blob_by_name("labels");
		blobLoss = net->blob_by_name("loss");
		blobOut  = net->blob_by_name("out");
		
		test_net = solver->test_nets()[0];

		// basic checking for minimal functioning
		CHECK(test_net->has_blob("data"));	
		CHECK(test_net->has_blob("labels"));	
		CHECK(test_net->has_blob("loss"));			
		CHECK(test_net->has_blob("out"));		
	
		test_blobData = test_net->blob_by_name("data");
		test_blobClip = test_net->blob_by_name("clip");
		test_blobLabel = test_net->blob_by_name("labels");
		test_blobLoss = test_net->blob_by_name("loss");
		test_blobOut  = test_net->blob_by_name("out");

		train_batch_size = blobData->shape(0);
		validate_batch_size = test_blobData->shape(0);

//		CHECK_EQ( train_set_size % train_batch_size, 0) << "train set size must be must be multiple of train batch size";
//		CHECK_EQ( validate_set_size % validate_batch_size, 0) << "validate set size must be must be multiple of validate batch size";

		train_batch_num = train_set_size / train_batch_size;
		validate_batch_num = validate_set_size / validate_batch_size;

		// input state size checks
		if( command_feedback && out_size == 1 ) 
			state_sequence_size = averaged_ranges_size + 3;
		else if( command_feedback && out_size == 2 ) 
			state_sequence_size = averaged_ranges_size + 4;
		else
			state_sequence_size = averaged_ranges_size + 2;
					
		CHECK_EQ( state_sequence_size, blobData->shape(1) ) << "train net: supposed input sequence size check failed";
		CHECK_EQ( state_sequence_size, test_blobData->shape(1) ) << "validate net: supposed input sequence size check failed";

		// output size checks
		CHECK_EQ(out_size, blobOut->shape(1) ) << " supposed output size and net loaded output must equal";
		CHECK_EQ(blobOut->shape(1), blobLabel->shape(1)) << "train net: output size and labels size must match"; 
		CHECK_EQ(test_blobOut->shape(1), test_blobLabel->shape(1)) << "test net: output size and labels size must match";
		
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
		LOG(INFO) << "TRAIN INFO: set size: " << train_set_size << " batch size: " << train_batch_size;
		LOG(INFO) << "VALIDATE INFO: set size: " << validate_set_size; 
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
				    " PARAMETERS: \n TRAIN_SIZE = %d \n VALID_SIZE = %d \n"
				    " train_batch_size = %d \n iter_size = %d \n batch_updates = %d \n "
				    " base_learning_rate = %.5f \n"
				    " weight_decay = %f \n "
				    " input_size = %d \n "
				    " command_feedback = %d \n "
				    " tail_type = %s \n "
				    " loss_data = [ \n ",
				     net->name().c_str(), local->tm_mon+1, 
				     local->tm_mday, local->tm_hour, local->tm_min,
				     train_set_size, validate_set_size, 
				     train_batch_size, iter_size, batch_updates, 
				     solver->param().base_lr(),
				     solver->param().weight_decay(),
					state_sequence_size,
					command_feedback,
					tail_type.c_str());
	
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

			if( i % state_sequence_size == 0 ) {
				blobClip->mutable_cpu_data()[i] = 0;
			}
			else {
				blobClip->mutable_cpu_data()[i] = 1;
			}
		}
	
		// populate validating clip blob 
		for(int i = 0; i < validate_batch_size; i++) {

			if( i % state_sequence_size == 0 ) {
				test_blobClip->mutable_cpu_data()[i] = 0;
			}
			else {
				test_blobClip->mutable_cpu_data()[i] = 1;
			}
		}
	
		FLAGS_minloglevel = 1;

		int SHOW_EPOCH_LOG = 1;
		int validation_test = 0;
		int epoch = 1;		

		float Test_loss = 0.0f;

		while( ros::ok() && epoch < epochs ) { // training process
		
			TRAIN = true;

			float Train_loss = 0.0f;	
		
			for(int k = 1; k < train_batch_num; k++) { // training batch 

				solver->Step(batch_updates);

				Train_loss += blobLoss->mutable_cpu_data()[0];

//				char answer;
//				cout << "TRAINING: Want to check batch output? (y/n)" << endl;
//				cin >> answer;
//				if ( answer == 'y' ) {

//				for(int i = 0; i < train_batch_size; i++) {  

//					printf("Batch %d  sample %d  State input sequence: ", k, i );
//					for(int j = 0; j < state_sequence_size; j++) {
//						printf(" %.4f  ",  blobData->mutable_cpu_data()[i * state_sequence_size + j]);
//					}
//					getchar();
//					cout << endl;

//					printf("Batch %d  sample %d  NET OUTS: ", k, i);   
//					for(int l=0; l < blobOut->channels(); l++) { 
//		
//						printf(" %.4f   ", blobOut->mutable_cpu_data()[i * blobOut->channels() + l]);
//			
//					}
//					cout << endl;

//					printf("Batch %d  sample %d  LABELS OUTS: ", k, i); 
//					for(int l=0; l < blobLabel->channels(); l++) {  
//		
//						printf(" %.4f  ", blobLabel->mutable_cpu_data()[i * blobLabel->channels() + l]);
//			
//					}
//					cout << endl;

//					printf("Batch %d  sample %d  Loss: %.5f \n", k, i, blobLoss->mutable_cpu_data()[0] );
//		
//					cout << "Wanna pass forward? (y/n)" << endl;
//					cin >> answer;

//					if(answer == 'y')
//						break;

//				}
//					

//				}

			}
		
			Train_loss /= train_batch_num;

			epoch++;
 
			if ( SHOW_EPOCH_LOG ) {
				LOG(WARNING) << "TRAIN EPOCH: " << epoch 
	 				        << " AVERAGE LOSS: " << Train_loss;
			}
		
			plot = fopen(matlab_plot.c_str(), "a");
			if( plot == NULL ) {
				cout << "Plot file opening failed.\n";
				exit(1);
			}
			fprintf(plot, "    %.4f   ", Train_loss); 
			fclose(plot);

			if( epoch % val_freq == 0 ) {

				TRAIN = false;
		
				validation_test++;

				Test_loss = 0;				

				UpdateTestNet();
			
				for(int k=1; k <= validate_batch_num; k++) { // validation test 

					test_net->Forward();	
		
					Test_loss += test_blobLoss->mutable_cpu_data()[0];			

//					char answer;
//					cout << "VALIDATING: Want to check batch output? (y/n)" << endl;
//					cin >> answer;
//					if ( answer == 'y' ) {

//						for(int i = 0; i < validate_batch_size; i++) {  

//						printf("Batch %d  sample %d  State input sequence: ", k, i );
//						for(int j = 0; j < state_sequence_size; j++) {
//							printf(" %.4f  ",  test_blobData->mutable_cpu_data()[i * state_sequence_size + j]);
//						}
//						getchar();
//						cout << endl;

//						printf("Batch %d  sample %d  NET OUTS: ", k, i);   
//						for(int l=0; l < blobOut->channels(); l++) { 
//		
//							printf(" %.4f   ", test_blobOut->mutable_cpu_data()[i * test_blobOut->channels() + l]);
//			
//						}
//						cout << endl;

//						printf("Batch %d  sample %d  LABELS OUTS: ", k, i); 
//						for(int l=0; l < blobLabel->channels(); l++) {  
//		
//							printf(" %.4f  ", test_blobLabel->mutable_cpu_data()[i * test_blobLabel->channels() + l]);
//			
//						}
//						cout << endl;

//						printf("Batch %d  sample %d   Test Loss: %.5f \n", k, i, test_blobLoss->mutable_cpu_data()[0] );
//		
//						cout << "Wanna pass forward? (y/n)" << endl;
//						cin >> answer;

//						if(answer == 'y')
//							break;

//				     }
//					

//					}


			
				
											
				} // end validation test 
			

			Test_loss /=  validate_batch_num;
		
			LOG(WARNING) << "VALIDATION TEST: " << validation_test 
				        << "  AVERAGE LOSS: "  << Test_loss;

			}
			
			plot = fopen(matlab_plot.c_str(), "a");
			if( plot == NULL ) {
				cout << "Plot file opening failed.\n";
				exit(1);
			}
			fprintf(plot, "    %.4f  ; \n", Test_loss); 
			fclose(plot);


		}

		
		
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


