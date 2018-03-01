

#include <neural_network_planner/train_database.h>


int main(int argc, char **argv ) {

ros::init(argc, argv, "train_database_node");
	
std::string process_name = "train_database";
neural_network_planner::TrainDatabase train_db(process_name);


}
