
#ifndef _ASTAR_MOD_H
#define _ASTAR_MOD_H

#include <costmap_2d/costmap_2d.h>
#include <global_planner_mod/planner_core.h>
#include <global_planner_mod/expander.h>
#include <vector>
#include <algorithm>

namespace global_planner_mod {

/** 
 * @stuct Node
 * @brief A* star node
 */
struct Node
{

	double G, H;
	int x, y;  ///< Here considered in Map coordinates >
	Node *parent;

	// @brief return the score of a A* node
	// @param G path cost from source node to current node
	// @param H heuristic path cost from current to goal node
	// @return sum of the two costs
	double getAstarScore();

	// @brief create empty node
	Node();
	// @brief create a new node given the coordinates
	// @param new_x x-axis coordinate
	// @param new_y y-axis coordinate
	Node(int new_x, int new_y);
	Node(int new_x, int new_y, Node *new_parent);

};


class Index {
    public:
        Index(int a, float b) {
            i = a;
            cost = b;
        }
        int i;
        float cost;
};

struct greater1 {
        bool operator()(const Index& a, const Index& b) const {
            return a.cost > b.cost;
        }
};

class AStarExpansion : public Expander {
    public:
        AStarExpansion(PotentialCalculator* p_calc, int nx, int ny, double move_rad_distance);
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                float* potential);

	   int calculatePath( costmap_2d::Costmap2D* costmap_, double start_x, 
			   double start_y, double goal_x, double goal_y, int cycles,
                           float* potential, std::vector< std::pair<float,float> >& path_found,
			   int& nodes_expanded, bool enforce_to_bounds);

	   double getCellCost(costmap_2d::Costmap2D* costmap_, int x, int y);

    private:
        void add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y);
        std::vector<Index> queue_;

      /* below there are parameters dedicated to A* algorithm basic version (adjacent nodes expanded)
       * custom implementation, similar to ASCII maps simulation environment version
       */  
	
	  ros::NodeHandle astar_global;
	  ros::Publisher marker_pub;

	  using NodeSet = std::vector<Node*>;
	
	  NodeSet openSet; ///< @brief set of open nodes >
	  NodeSet closedSet; ///< @brief set of closed nodes >

	  std::vector<double> steering_angles; ///< @brief allowed steering angle to search for path

	  /**
           *@brief Set the steering angles allowed
	   */
	  void setSteeringAngles(std::vector<double>& steering_angles, double move_angle_distance);

	  /**
	   * @return if present, return index position of curr in nodes
	   *		 if not present, return nodes size
  	   */
	  int  find_index(NodeSet& nodes, int new_x, int new_y);

	  /**
	   * @return if present, return pointer to node with searched coordinates
	   *		 if not present, return nullptr
           */		
	  Node* find_node(NodeSet& nodes, int new_x, int new_y);
	
	  /**
           * @brief compute modified euclidean distance between source and target
           *        as a heuristic distance
	   */ 
	  float euclidean_mod(Node *source, Node *target);

	  /**
           * @brief free the NodeSet from memory
           */
	  void releaseNodes(NodeSet& nodes);

		

};

} //end namespace global_planner
#endif

