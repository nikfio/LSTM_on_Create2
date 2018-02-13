
#include <neural_network_planner/build_database.h>


int main(int argc, char **argv) {

ros::init(argc, argv, "build_database");
	
std::string base_name = "database_lab";
neural_network_planner::BuildDatabase build_db(base_name);

return(0);

}

