

#include <neural_network_planner/train_online.h>


int main(int argc, char **argv ) {

ros::init(argc, argv, "train_online_node");
	
std::string process_name = "train_online";
neural_network_planner::TrainOnline train_onl(process_name);


}
