
#include <neural_network_planner/build_database.h>
#include <glog/logging.h>

int main(int argc, char **argv) {

ros::init(argc, argv, "build_database");

google::InitGoogleLogging(argv[0]);
	
std::string base_name = "database_lab";
neural_network_planner::BuildDatabase build_db(base_name);

return(0);

}

