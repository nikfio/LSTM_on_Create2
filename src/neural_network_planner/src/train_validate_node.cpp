
#include <neural_network_planner/train_validate.h>


int main(int argc, char **argv) {

ros::init(argc, argv, "train_validate");

std::string name = "rnn_train_process";
neural_network_planner::TrainValidateRNN LSTM_training(name);

ros::spin();

return(0);


}
