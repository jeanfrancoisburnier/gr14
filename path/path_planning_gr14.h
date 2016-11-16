/*! 
 * \author Group 14
 * \file path_planning_gr14.h
 * \brief path-planning algorithm
 */



#ifndef _PATH_PLANNING_GR14_H_
#define _PATH_PLANNING_GR14_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"

#include <vector>
#include <array>


#define SQUARE_SIZE 50 //[mm] dimensions of the grid's squares
#define X 0 	//index of the x position in the array coordinates
#define Y 1 	//index of the y position in the array coordinates

using namespace std; //to be able to use array


NAMESPACE_INIT(ctrlGr14);

/// path-planning main structure
struct PathPlanning
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};






//rectangle delimited by two extreme corner (first : top left, second : bottom right) 
struct Obstacles 
{
	array<float, 2> first_corner;
	array<float, 2> second_corner;
};





//Ede object contained in a Node Object
class Edge
{
private:
	int id;					//current edge's id
	float weight;			//correspond to the length of the edge (twos differents values for us 1 or sqrt(2))

public:
	int edge_get_id()
	{
		return id;
	}

	float edge_get_weight()
	{
		return weight;
	}


	Edge::Edge(int id_edge, float weight_edge)
	{
		id = id_edge;
		weight = weight_edge;

	}

	Edge::~Edge()
	{}
};






//Node object containing among other things eight Edge objects
class Node
{
private:
	int id;							//current node's id
	int previous_node_id;			//id of the previous node visited
	bool visited = false;			//true if this node havs been already during this pathplanning and false otherwise 
	bool free_position;				//indicate if the node is accessible or not
	array<float, 2> coordinates;	//coordinates of the node
	vector<Edge> direct_edges;	//differents edges of the Node (connection to other nodes)

public:
	int distance_to_start;	//total distance already travelled from the start (to see if we go in the good direction)


	Node::Node(int id_node, int* ptr_total_edge_index, bool free_init)
	{
		id = id_node;
		free_position = free_init;
		
		//coordinates[X] = calcul with the id_repartition and the number of squares
		//coordinates[Y] = calcul with the id_repartition and the number of squares

		//create the right amount of edeges : number edges --> calcul the particular cases
		//int number edges = 8;
		//set the number of edges so the size of vector direct_edges

		//Do the initialization of the different edges

	}

	Node::~Node()
	{}
};









PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
