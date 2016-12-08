/*! 
 * \author Group 14
 * \file node.h
 * \Edge class
 */
#include "edge_gr14.h"

Edge::Edge(int id_edge, int id_end_node, float weight_edge) : id(id_edge), id_connected_node(id_end_node), weight(weight_edge)
{
	;
}

Edge::~Edge()
{}


int Edge::edge_get_id_connected_node()
{
	return id_connected_node;
}

float Edge::edge_get_weight()
{
	return weight;
}


