/*! 
 * \author Group 14
 * \file node.h
 * \Node class + one function
 */
#ifndef _NODE_GR14_H_
#define _NODE_GR14_H_

#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"

#include "edge_gr14.h"
#include <vector>
#include <array>

#define X 0 	//index of the x position in the array coordinates
#define Y 1 	//index of the y position in the array coordinates

#define SQUARE_SIZE 50 //[mm] dimensions of the grid's squares who've got a Node at their center
 					   //Has to divide of 2*MAX_X and 2*MAX_Y because we want a uniforme grid

#define MAX_X 850 //(+-) Value in mm of the distance of the map along the x axis 
 				   //substract to the dimensions of the robot + a security range 
 				   //to avoid colisions (Robot then considered as a material point)

#define MAX_Y 1350 //Same as x but  along the Y axis

#define MAX_NB_EDGES 8 //top, left, right, bottom and the other four diagonals

const float square_length = SQUARE_SIZE * 0.001; //conversion into [m] of the dimensions of the grid's squares who've got a Node at their center
const float peak_x = MAX_X * 0.001;			     //[m]
const float peak_y = MAX_X * 0.001;			     //[m]

//Number of nodes along the x/y axis
const int NB_X = 2 * MAX_X / SQUARE_SIZE;
const int NB_Y = 2 * MAX_Y / SQUARE_SIZE;

using namespace std; //to be able to use some identifier proper to C++




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

	float distance_to_goal;

	void node_creation_edges(int state);
	int node_identify_state(int id_n);
	void node_build_valid_edge(int index, array<int, MAX_NB_EDGES> shift_i);

public:
	float distance_to_start;	//total distance already travelled from the start (to see if we go in the good direction)

	Node(int id_node, bool free_init, float x_p, float y_p);
	~Node();

	void node_set_previous_node_id(int id_prev);
	void node_set_distance_to_goal(float x_g, float y_g);

	array<float, 2> node_get_coordinates();
	vector<Edge> node_get_edges();
	int node_get_previous_node_id();						
	bool node_get_visited();
	bool node_get_free_position();
	float node_get_distance_to_goal();
};


int node_find_closest_node(float x_p, float y_p);//return the id of the closest Node


NAMESPACE_CLOSE();

#endif