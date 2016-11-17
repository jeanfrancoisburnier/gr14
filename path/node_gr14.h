/*! 
 * \author Group 14
 * \file node.h
 * \Node class
 */

#ifndef _NODE_GR14_H_
#define _NODE_GR14_H_

 #include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"

#include "edge_gr14.h"
#include <vector>
#include <array>

#define SQUARE_SIZE 50 //[mm] dimensions of the grid's squares
#define X 0 	//index of the x position in the array coordinates
#define Y 1 	//index of the y position in the array coordinates
#define MAX_NB_EDGES 8 //top, left, right, bottom and the other four diagonals


using namespace std; //to be able to use array

NAMESPACE_INIT(ctrlGr14);



//Node object containing among other things eight Edge objects
class Node
{
private:
	int id;										//current node's id
	int previous_node_id;						//id of the previous node visited
	bool visited = false;						//true if this node havs been already during this pathplanning and false otherwise 
	bool free_position;							//indicate if the node is accessible or not
	array<float, 2> coordinates;				//coordinates of the node
	vector<Edge> direct_edges;	//differents edges of the Node (connection to other nodes)

public:
	int distance_to_start;	//total distance already travelled from the start (to see if we go in the good direction)

	Node(int id_node, int* ptr_total_edge_index, bool free_init);
	~Node();

	array<float, 2> node_get_coordinates();
	vector<Edge> node_get_edges();
	int node_get_previous_node_id();						
	bool node_get_visited();
	bool node_get_free_position();

	void set_previous_node_id(int id_prev);
};



NAMESPACE_CLOSE();

#endif