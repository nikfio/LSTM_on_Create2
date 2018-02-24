
#include <neural_network_planner/train_validate.h>
#include <glog/logging.h>

int main(int argc, char **argv) {

ros::init(argc, argv, "train_validate_node");

google::InitGoogleLogging(argv[0]);

std::string name = "rnn_train_process";
neural_network_planner::TrainValidateRNN training_proc(name);


return(0);


}
