/*! 
 * \author Group 14
 * \file node.h
 * \Node class + one function
 */
#include "node_gr14.h"
#include "useful_gr14.h"
#include "math.h"
#include <iostream>

typedef enum
{
	LEFT_TOP_CORNER = 0,
	TOP_BORDER = 1, //without corners
	RIGHT_TOP_CORNER = 2,
	RIGHT_BORDER = 3,
	RIGHT_BOTTOM_CORNER = 4,
	BOTTOM_BORDER = 5,
	LEFT_BOTTOM_CORNER = 6,
	LEFT_BORDER = 7,
	INSIDE = 8 
} position_node_t;

NAMESPACE_INIT(ctrlGr14);


Node::Node(int id_node, bool free_init, float x_p, float y_p): id(id_node), free_position(free_init), heuristic_value(H_VALUE_INIT)
{		
	coordinates[X]=x_p;
	coordinates[Y]=y_p;
	
	int state;
	state = node_identify_state(id);
	node_creation_edges(state);
}


Node::~Node()
{}



//When a Node is created we have to be careful if it's in the border of the map 
//Because it's Edges will be pointing in a forbidden zone, that's why this function 
//check attribute a state to the Node defined by id_n
int Node::node_identify_state(int id_n)
{
	if( id_n == 0 )//Node in the top left corner
	{
		return LEFT_TOP_CORNER;
	}
	else if( id_n < (NB_X - 1) )//Nodes at the top border (except corners) 
	{
		return TOP_BORDER;
	}
	else if( id_n == (NB_X - 1) )//Node at the top right corner
	{
		return RIGHT_TOP_CORNER;
	}
	else if( (id_n % (NB_X - 1)) == 0 )//Nodes at the right border (except corners)
	{
		return RIGHT_BORDER;
	}
	else if( id_n == (NB_X*NB_Y - 1) )//Node at the bottom right corner
	{
		return RIGHT_BOTTOM_CORNER;
	}
	else if( id_n > ((NB_Y-1) * NB_X) )//Nodes at the bottom border (except corners)
	{
		return BOTTOM_BORDER;
	}
	else if( id_n == ((NB_Y-1) * NB_X) )//Node at the bottom left corner
	{
		return LEFT_BOTTOM_CORNER;
	}
	else if( (id_n % NB_X) == 0 )//Nodes at the left border (except corners)
	{
		return LEFT_BORDER;
	}
	else//node not on the map border
	{
		return INSIDE;
	}
}



//------ Do the initialization of the different edges ------//
void Node::node_creation_edges(int state)
{
	array<int, MAX_NB_EDGES> shift_index = {id-NB_X, id-(NB_X -1), id+1, id+(NB_X+1), id+NB_X, id+(NB_X-1), id-1, id-(NB_X+1)};
	//shift (value to add to the actual Node's id) in our grid to obtain the id of the destination Node
	//in the good order : top, top right, right, bottom right, bottom, bottom left, left, top left

	direct_edges.reserve(MAX_NB_EDGES);//Each Node will have 8 Edges

	//Edges pointing outside the map are attribuated a weight of 0 and an id destination of -1
	//We give to the Edge class constructor the Edge's id, the pointed Node's id and the weight of the Edge (length)
	switch(state)
	{
		case LEFT_TOP_CORNER :
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				if( i<=1 || i>4 )//edges pointing outside the map
				{
					direct_edges.push_back (Edge (i, -1, 0));
				}
				else//edges pointing in the map
				{
					node_build_valid_edge(i, shift_index);
				}
			}
			break;

		case TOP_BORDER ://(except corners) 
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				if( i<=1 || i==7 )//edges pointing outside the map
				{
					direct_edges.push_back (Edge (i, -1, 0));
				}
				else//edges pointing in the map
				{
					node_build_valid_edge(i, shift_index);
				}
			}
			break;

		case RIGHT_TOP_CORNER :
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				if( i<4 || i==7 )//edges pointing outside the map
				{
					direct_edges.push_back (Edge (i, -1, 0));
				}
				else//edges pointing in the map
				{
					node_build_valid_edge(i, shift_index);
				}
			}
			break;

		case RIGHT_BORDER : //(except corners)
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				if( i>=1 && i<=3 )//edges pointing outside the map
				{
					direct_edges.push_back (Edge (i, -1, 0));
				}
				else//edges pointing in the map
				{
					node_build_valid_edge(i, shift_index);
				}
			}
			break;

		case RIGHT_BOTTOM_CORNER : 
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				if( i>=1 && i<=5 )//edges pointing outside the map
				{
					direct_edges.push_back (Edge (i, -1, 0));
				}
				else//edges pointing in the map
				{
					node_build_valid_edge(i, shift_index);
				}
			}
			break;

		case BOTTOM_BORDER : //(except corners) 
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				if( i>=3 && i<=5 )//edges pointing outside the map
				{
					direct_edges.push_back (Edge (i, -1, 0));
				}
				else//edges pointing in the map
				{
					node_build_valid_edge(i, shift_index);
				}
			}
			break;

		case LEFT_BOTTOM_CORNER :
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				if( i>=3 )//edges pointing outside the map
				{
					direct_edges.push_back (Edge (i, -1, 0));
				}
				else//edges pointing in the map
				{
					node_build_valid_edge(i, shift_index);
				}
			}
			break;

		case LEFT_BORDER : // (except corners) 
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				if( i>=5 )//edges pointing outside the map
				{
					direct_edges.push_back (Edge (i, -1, 0));
				}
				else//edges pointing in the map
				{
					node_build_valid_edge(i, shift_index);
				}
			}
			break;

		case INSIDE ://node not on the map border
			for(int i = 0; i < MAX_NB_EDGES; i++)
			{
				node_build_valid_edge(i, shift_index);
			}
			break;
	}
}


void Node::node_build_valid_edge(int index, array<int, MAX_NB_EDGES> shift_i)
{
	if( (index % 2) == 0 )//indice pair (top, left, bottom, right)
	{
		direct_edges.push_back (Edge (index, shift_i[index], 1));
	}
	else//indice impair (the 4 diagonals)
	{
		direct_edges.push_back (Edge (index, shift_i[index], sqrt(2)));
	}
}



// ------ Settings ----- //

void Node::node_set_previous_node_id(int id_prev)
{
	previous_node_id=id_prev;
}

void Node::node_set_distance_to_goal(array<float, 2> coord_g)
{
	distance_to_goal = sqrt( pow((coord_g[X] - coordinates[X]),2) + pow((coord_g[Y] - coordinates[Y]),2) );
}

void Node::node_set_distance_to_start(array<float, 2> coord_s)
{
	distance_to_goal = sqrt( pow((coord_s[X] - coordinates[X]),2) + pow((coord_s[Y] - coordinates[Y]),2) );
}

void Node::node_set_heuristic_value(float h_value)
{
	heuristic_value = h_value;
}


// ------ Getting ----- //

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

float Node::node_get_distance_to_goal()
{
	return distance_to_goal;
}

float Node::node_get_distance_to_start()
{
	return distance_to_start;
}

float Node::node_get_heuristic_value()
{
	return heuristic_value;
}




// ----- This function isn't in the class  ------ //
//Find the id of the closest node to a point described by its coordinates.
int node_find_closest_node(float x_p, float y_p)//return the id of the closest Node
{
	if( x_p>peak_x )
	{
		cout << "Warning your coordinates are outside the range of the robot\n" << endl;
		x_p = peak_x;
	}
	else if( x_p<-peak_x )
	{
		cout << "Warning your coordinates are outside the range of the robot\n" << endl;
		x_p = -peak_x;
	}

	if( y_p>peak_y )
	{
		cout << "Warning your coordinates are outside the range of the robot\n" << endl;
		y_p = peak_y;
	}
	else if( x_p<-peak_x )
	{
		cout << "Warning your coordinates are outside the range of the robot\n" << endl;
		y_p = -peak_y;
	}


	float id_x=0.0;
	float id_y=0.0;
	float id_temp=0.0;

	id_x = (peak_x - square_length/2 + x_p) / square_length;
	id_y = (peak_y + square_length/2 - y_p) / square_length;
	id_temp = id_x + NB_X * id_y;

	if( id_temp - (int)(id_temp) > 0.5 )
	{
		return (int)(id_temp + 1);
	}
	else
	{
		return (int)(id_temp);
	}
}

NAMESPACE_CLOSE();

