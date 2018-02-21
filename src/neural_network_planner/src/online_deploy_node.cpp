


/* THE FINAL STAGE:
 * Launch the online deploy node for the network
 */


#include <neural_network_planner/online_deploy.h>


int main(int argc, char **argv) 
{

ros::init(argc, argv, "online_deploy_node");
	
std::string process_name = "online_deployment";
neural_network_planner::OnlineDeploy online_test(process_name);


return(0);

}
