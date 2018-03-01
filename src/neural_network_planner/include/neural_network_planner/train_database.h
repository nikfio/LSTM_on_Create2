
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


  class TrainDatabase 
  {

     public:
     
	TrainDatabase(std::string& process_name);

	~TrainDatabase();

     private:

	ros::NodeHandle private_nh;

     /*
	 * @brief predefined blobs to handle the training process
	 *        standard elements for every recurrent network
      */
	 

     boost::shared_ptr<caffe::Solver<float> > solver;
     boost::shared_ptr<caffe::Net<float> > net;
	boost::shared_ptr<caffe::Net<float> > test_net;

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
	int train_batch_size, train_set_size, train_batch_num; 	
	int validate_batch_size, validate_set_size, validate_batch_num;
	int state_sequence_size, epochs;
	int time_sequence, averaged_ranges_size;

	void UpdateTestNet();

  };



} // namespace neural_network_planner


#endif
