/* Functions related to nodes management by the A* local agent
 */

#include <global_planner_mod/astar_mod.h>

using global_planner_mod::Node;


namespace global_planner_mod {



Node::Node() {

	G = 0;
	H = 0;

};

Node::Node(int new_x, int new_y) {

	x = new_x;
	y = new_y;
	G = 0;
	H = 0;
	parent = nullptr;

};

Node::Node(int new_x, int new_y, Node *new_parent) {

	x = new_x;
	y = new_y;
	G = 0;
	H = 0;
	parent = new_parent;

};



double Node::getAstarScore() {

	return G + H;

};

int  AStarExpansion::find_index(NodeSet& nodes, 
				int new_x,
				int new_y) {

	int i;
	for (i = 0; i<nodes.size(); i++) {
		    if ( nodes[i]->x == new_x && nodes[i]->y == new_y) {
		        break;
		    }
	}
	
	return i;
	
};

Node* AStarExpansion::find_node(NodeSet& nodes, 
				int new_x,
				int new_y)
{
    for (int i =0; i<nodes.size(); i++) {
        if ( nodes[i]->x == new_x && nodes[i]->y == new_y ) {
            return nodes[i];
        }
    }
    return nullptr;
};

float AStarExpansion::euclidean_mod(Node *source, Node *target)
{
    float delta_x = target->x - source->y;
    float delta_y = target->y - source->y; 
    return static_cast<float>( 10 * sqrt(pow(delta_x, 2) + pow(delta_y, 2)) );
};


void AStarExpansion::releaseNodes(NodeSet& nodes)
{

    for(int i=0; i<nodes.size(); i++) {
        nodes.pop_back();
    }
    
   
};




} // namespace astar_local_planner
