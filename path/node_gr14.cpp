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
	TOP_LEFT_CORNER = 0,
	TOP_BORDER = 1, //without corners
	TOP_RIGHT_CORNER = 2,
	RIGHT_BORDER = 3,
	BOTTOM_RIGHT_CORNER = 4,
	BOTTOM_BORDER = 5,
	BOTTOM_LEFT_CORNER = 6,
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

Node::Node()
{}
Node::~Node()
{}



//When a Node is created we have to be careful if it's in the border of the map 
//Because it's Edges will be pointing in a forbidden zone, that's why this function 
//check attribute a state to the Node defined by id_n
int Node::node_identify_state(int id_n)
{
	if( id_n == 0 )//Node in the top left corner
	{
		//printf("TOP_LEFT_CORNER\n");
		return TOP_LEFT_CORNER;
	}
	else if( id_n == (NB_X - 1) )//Node at the top right corner
	{
		//printf("TOP_RIGHT_CORNER\n");
		return TOP_RIGHT_CORNER;
	}
	else if( id_n == (NB_X*NB_Y - 1) )//Node at the bottom right corner
	{
		//printf("BOTTOM_RIGHT_CORNER\n");
		return BOTTOM_RIGHT_CORNER;
	}
	else if( id_n == ((NB_Y-1) * NB_X))//Node at the bottom left corner
	{
		//printf("BOTTOM_LEFT_CORNER\n");
		return BOTTOM_LEFT_CORNER;
	}
	else if( id_n < (NB_X - 1) )//Nodes at the top border (except corners) 
	{
		//printf("TOP_BORDER\n");
		return TOP_BORDER;
	}
	else if( (id_n+1) % (NB_X) == 0)//Nodes at the right border (except corners)
	{
		//printf("RIGHT BORDER\n");
		return RIGHT_BORDER;
	}
	
	else if( id_n > ((NB_Y-1) * NB_X) )//Nodes at the bottom border (except corners)
	{
		//printf("BOTTOM_BORDER\n");
		return BOTTOM_BORDER;
	}
	else if( (id_n % NB_X) == 0 )//Nodes at the left border (except corners)
	{
		//printf("LEFT_BORDER\n");
		return LEFT_BORDER;
	}
	else//node not on the map border
	{
		//printf("IN\n");
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
		case TOP_LEFT_CORNER :
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

		case TOP_RIGHT_CORNER :
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

		case BOTTOM_RIGHT_CORNER : 
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

		case BOTTOM_LEFT_CORNER :
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

void Node::node_set_distance_to_start(float dist)
{
	distance_to_start = dist;
}

void Node::node_set_heuristic_value(float h_value)
{
	heuristic_value = h_value;
}

void Node::node_set_visited(bool visit)
{
	visited = visit;
}

void Node::node_set_free_position(bool free_pos)
{
	free_position = free_pos;
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

int Node::node_get_id()
{
	return id;
}


/*! \ Method to scan through the edges of a node
 * 
 * \param[in,out] List of all the nodes available so that we can modify the list with a*
 * \param[in] goal node so that we can compute the distance to goal for the new nodes
 * returns a list of node ids that have not been visited by the path finding algorithm yet
 */
vector<int> Node::scan_edges(vector<Node>& node_list,Node goal)
{
	vector<int> ids;
	int next_node_id;
	for (auto edge:node_get_edges())
	{
		//check if the destination node exists if not continue to the next one
		if(edge.edge_get_weight() == 0)
		{
			continue;
		}

		next_node_id = edge.edge_get_id_connected_node();

		//check if the destination node has already been visited or if it is occupied by an obstacle
		//if it returns true we go onto the next node
		if(node_list[next_node_id].node_get_visited() == true || node_list[next_node_id].node_get_free_position() == OCCUPIED)
		{
			continue;
		}

		/*
		* Set the distance to start of the new node, the distance to goal, it's heuristic function value, the id of 
		* the node we arrived here from and finally sets the visited boolean to true 
		*/
		node_list[next_node_id].node_set_distance_to_start(node_get_distance_to_start() + edge.edge_get_weight());
		node_list[next_node_id].node_set_distance_to_goal(goal.node_get_coordinates());
		node_list[next_node_id].node_set_heuristic_value(node_list[next_node_id].node_get_distance_to_start() 
														+ node_list[next_node_id].node_get_distance_to_goal());
		node_list[next_node_id].node_set_previous_node_id(node_get_id());
		node_list[next_node_id].node_set_visited(true);
		ids.push_back(next_node_id);
	} 
	return ids;
}


// ----- These functions aren't in the class  ------ //
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
	id_y = (peak_y - square_length/2 - y_p) / square_length;
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

//-------operator used in the priority queue-------//

//
bool compare_heuristic::operator()(Node& n1, Node& n2) 
{
       return n1.node_get_heuristic_value() > n2.node_get_heuristic_value();
}


NAMESPACE_CLOSE();

