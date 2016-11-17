/*! 
 * \author Group 14
 * \file node.h
 * \Node class
 */
#include "node_gr14.h"
#include "useful_gr14.h"
#include "math.h"


NAMESPACE_INIT(ctrlGr14);


Node::Node(int id_node, int* ptr_total_edge_index, bool free_init): id(id_node), free_position(free_init)
{		
	//coordinates[X] = calcul with the id_repartition and the number of squares
	//coordinates[Y] = calcul with the id_repartition and the number of squares

	
	
	*ptr_total_edge_index += MAX_NB_EDGES;
	array<int, MAX_NB_EDGES> shift_index = {0, 0, 0, 0, 0, 0 ,0 ,0}; //change the 0s by their true value
	//warning, take into account when we exit the map
	//shift in our grid to obtain the id of the destination node from the actual one
	//in the good order : top, top right, right, bottom right, bottom, bottom left, left, top left


	//Do the initialization of the different edges
	direct_edges.reserve(MAX_NB_EDGES);
	for(int i = 0; i < MAX_NB_EDGES; i++)
	{
		if((i % 2) == 0)//indice pair (top, left, bottom, right)
		{
			direct_edges.push_back (Edge (*ptr_total_edge_index + 1 + i, shift_index[i], 1));
		}
		else//indice impair (the 4 diagonals)
		{
			direct_edges.push_back (Edge (*ptr_total_edge_index + 1 + i, shift_index[i], sqrt(2)));
		}
	}

}

Node::~Node()
{}


void Node::set_previous_node_id(int id_prev)
{
	previous_node_id=id_prev;
}



array<float, 2> Node::node_get_coordinates()
{
	return coordinates;
}


vector<Edge> Node::node_get_edges()
{
	return direct_edges;
}


int Node::node_get_previous_node_id()
{
	return previous_node_id;
}						
	

bool Node::node_get_visited()
{
	return visited;
}



bool Node::node_get_free_position()
{
	return free_position;
}




NAMESPACE_CLOSE();

