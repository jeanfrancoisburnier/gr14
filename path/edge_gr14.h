/*! 
 * \author Group 14
 * \file node.h
 * \Edge class
 */

#ifndef _EDGE_GR14_H_
#define _EDGE_GR14_H_

#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"

#include <vector>
#include <array>

#define MAX_NB_EDGES 8 //top, left, right, bottom and the other four diagonals

using namespace std; //to be able to use array

NAMESPACE_INIT(ctrlGr14);


//Ede object contained in a Node Object
class Edge
{
private:
	int id;					//current edge's id
	float weight;			//correspond to the length of the edge (three differents values for us 1 or sqrt(2) or -1 if not a travelled edge)
	int id_connected_node;	//id to the destination Node
public:
	int edge_get_id_connected_node();
	float edge_get_weight();

	Edge(int id_edge, int id_end_node, float weight_edge);
	~Edge();
};




NAMESPACE_CLOSE();

#endif