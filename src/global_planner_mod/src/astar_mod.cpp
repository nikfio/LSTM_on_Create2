

#include <global_planner_mod/astar_mod.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/Marker.h>

#include <iostream>

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

using std::cout;
using std::endl;

namespace global_planner_mod {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys, double move_rad_distance) :
        Expander(p_calc, xs, ys) {

	setSteeringAngles(steering_angles, move_rad_distance);
	marker_pub = astar_global.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);
    }

    return false;
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    if (potential[next_i] < POT_HIGH)
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs(end_x - x) + abs(end_y - y);

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

/* Set the directions of adjacent nodes allowed to expand
 */
void AStarExpansion::setSteeringAngles(std::vector<double>& steering_angles, double move_angle_distance) {

		if( !steering_angles.empty() ) {	
			steering_angles.clear();
		}

		// converting to radians
		double rad_distance = (move_angle_distance * M_PI) / 180;
		
		steering_angles.push_back(0);
		int move_allowed = 1;

		double next_counterclock = rad_distance;
		
		while( next_counterclock <= fabs( M_PI - rad_distance) ) {
		
			steering_angles.push_back(next_counterclock);
			next_counterclock += rad_distance;
			move_allowed++;

		}
	
		int total_counterclock = move_allowed; // steering_angles.size()
		double next_clockwise;

		for(int i = 1; i < total_counterclock; i++) {

			next_clockwise = - steering_angles[i];
			steering_angles.push_back(next_clockwise);
			move_allowed++;		
		
		}

		steering_angles.push_back(M_PI);

//		for(int j = 0; j < steering_angles.size(); j++) {

//			LOG(INFO) << "Steering angle allowed: " << steering_angles[j];		
//		
//		}
		
	
		ROS_INFO("Total movements allowed: %d ", (int) steering_angles.size() );


	
};


/* Modified version of A* basic algorithm, produces directly a path as vector of
 * coordinates, stops at goal or at map limit if goal is out of map
 */

int AStarExpansion::calculatePath( costmap_2d::Costmap2D* costmap_, 
				   double start_x, double start_y,
				   double goal_x, double goal_y, int cycles,
                       float* potential, std::vector< std::pair<float,float> >& path_found,
				   int& nodes_expanded, bool enforce_to_bounds)

{

	unsigned int source_x, source_y, target_x, target_y;
	
	

	Node *SourceNode = new Node(source_x, source_y);
	Node *TargetNode;

	if( !costmap_->worldToMap(goal_x, goal_y, target_x, target_y) ) {
		ROS_WARN("Goal request is off legal bounds of global map, selecting alternative");
		
		int target_off_x, target_off_y;
		if( enforce_to_bounds ) {
			costmap_->worldToMapEnforceBounds(goal_x, goal_y, target_off_x, target_off_y);
			ROS_WARN("Enforcing goal coordinates to global map");
		}
		else {
			costmap_->worldToMapNoBounds(goal_x, goal_y, target_off_x, target_off_y);
			ROS_WARN("Goal coordinates conversion are not guaranteed to lie in legal bounds of global map");
		}
		
		TargetNode = new Node(target_off_x, target_off_y);		

	}
	else {
			TargetNode = new Node(target_x, target_y);	
	}	
	

	Node *current;
	Node *successor;

	openSet.push_back(SourceNode);

	double totalCost = 0;
	double resolution = costmap_->getResolution();

	bool map_limit_reached = false;

	visualization_msgs::Marker points;
	points.header.frame_id = "AStar_local_nodes_expanded";
	points.header.stamp = ros::Time::now();
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;	
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.2;   
	points.scale.y = 0.2; 
	points.color.r = 1.0;   
	points.color.a = 1.0;

	double x_i, y_i;
	
	while( !openSet.empty() ) {

	  current = openSet.front();

	  costmap_->mapToWorld(current->x, current->y, x_i, y_i);

	  //check the point on the trajectory for legality
//          double footprint_cost = footprintCost(x_i, y_i, theta_i);

	  //if the footprint hits an obstacle this trajectory is invalid
//	  if(footprint_cost < 0){
//		ROS_WARN("FOOTPRINT POSITION NOT VALID!");
//		traj.cost_ = -1.0;
//		break;
//	  }
//	  
	  for(int i = 0; i < openSet.size(); i++) {
	  	if( openSet[i]->getAstarScore() < current->getAstarScore() ) {
			current = openSet[i];
		}
	  }

          if(map_limit_reached){
	     ROS_DEBUG_NAMED("astar_global_mod", "trajectory searching cannot go beyond global map"); 
             break;
          }

	  closedSet.push_back(current);
	  if( find_index(openSet, current->x, current->y) < openSet.size() ) {
			ROS_DEBUG_NAMED("astar_global_mod", "erasing node in openSet: (%d, %d) ", current->x, current->y);		
        		openSet.erase(openSet.begin() + find_index(openSet, current->x, current->y));
	  }
	  else {
			ROS_DEBUG_NAMED("astar_global_mod", "Current node is not on OPEN list");
	  }

	  for (int i = 0; i < steering_angles.size(); i++) {

		   unsigned int exp_x, exp_y;

		   // sim_granularity or map resolution?
		   double next_x = cos( steering_angles[i] ) * resolution; 
		   double next_y = sin( steering_angles[i] ) * resolution;  

		   if( !costmap_->worldToMap(x_i + next_x, y_i + next_y, exp_x, exp_y) ) {
			ROS_DEBUG_NAMED("astar_global_mod", "Node expansion failed: : ( %.4f ; %.4f )", x_i + next_x, y_i + next_y);
			continue;
		   }
		  
             geometry_msgs::Point expanded;
		   expanded.x = x_i + next_x;
		   expanded.y = y_i + next_y;
		
		   ROS_DEBUG_NAMED("astar_global_mod", "ASTAR GLOBAL is expanding: ( %.4f ; %.4f )", expanded.x, expanded.y); 
		   points.points.push_back(expanded);
	
		   marker_pub.publish(points);
//		   getchar();
             if ( !getCellCost(costmap_, exp_x, exp_y) ) {
		 	ROS_INFO("Collision DETECTED");
		 	continue;
             }
			
		     
//		   MapCell cell = path_map_(exp_x, exp_y);

//		  // we don't want a path that goes off the known map
//		  // when path goes off the map cycle trajectory is finished
//		  if(cell.target_dist == path_map_.unreachableCellCosts()){
//			map_limit_reached = true;
//		        break;
//		  }

		  if ( find_node(closedSet, exp_x, exp_y) ) {
			continue;
		  }

		  ++nodes_expanded;

		  // Propagate cost as simple linear distance 
		  totalCost = current->G + hypot(next_x, next_y);

		  successor = find_node(openSet, exp_x, exp_y);
		  if (successor == nullptr ) {
			successor = new Node(exp_x, exp_y, current);		
			successor->G = totalCost;
			successor->H = euclidean_mod(successor, TargetNode);
			openSet.push_back(successor);		
		  }
		  else if (totalCost < successor->G) {
			successor->parent = current;
			successor->G = totalCost;
			successor->H = euclidean_mod(successor, TargetNode);
                  }

	      }
			
      }

      delete SourceNode;
      delete TargetNode;

      releaseNodes(openSet);
      releaseNodes(closedSet);

      std::vector<Node*> NodePath;
	
      while (current != nullptr) {
       	NodePath.push_back(current);
       	current = current->parent;
      }

      if(NodePath.size() == 1) {
	ROS_DEBUG_NAMED("astar_global_mod", "Given starting position, A* global mod did not find solution");
	return 0;
      }	
	
      std::reverse( NodePath.begin(), NodePath.end() );

      //clear the path to return, just in case
      path_found.clear();
      std::pair<float, float> add;    

      for(int i = 0; i < NodePath.size(); i++) {
	add.first = NodePath[i]->x;
	add.second = NodePath[i]->y;
		
	path_found.push_back(add);

      }

      return 1;


}

double AStarExpansion::getCellCost(costmap_2d::Costmap2D* costmap_, int x, int y){

    unsigned char cost = costmap_->getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    cout << x << " , " << y << "  pointCost: " << cost << "   " << LETHAL_OBSTACLE << INSCRIBED_INFLATED_OBSTACLE << "   " << NO_INFORMATION <<  endl;
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;

}

} //end namespace global_planner
