/*! 
 * \author Group 14
 * \file node.h
 * \Edge class
 */
#include "edge_gr14.h"
#include "useful_gr14.h"

NAMESPACE_INIT(ctrlGr14);


Edge::Edge(int id_edge, int id_end_node, float weight_edge) : id(id_edge), id_connected_node(id_end_node), weight(weight_edge)
{
	;
}

Edge::~Edge()
{}


int Edge::edge_get_id()
{
	return id;
}

float Edge::edge_get_weight()
{
	return weight;
}



NAMESPACE_CLOSE();
