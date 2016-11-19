/*! 
 * \author Group 14
 * \file node.h
 * \Node class
 */
#include "node_gr14.h"
#include "useful_gr14.h"
#include "math.h"
 #include <iostream>


NAMESPACE_INIT(ctrlGr14);


Node::Node(int id_node, bool free_init, float x_p, float y_p): id(id_node), free_position(free_init)
{		
	//coordinates[X] = calcul with the id_repartition and the number of squares
	//coordinates[Y] = calcul with the id_repartition and the number of squares
	coordinates[X]=x_p;
	coordinates[Y]=y_p;
	
	array<int, MAX_NB_EDGES> shift_index = {id-NB_X, id-(NB_X -1), id-1, id+1, id+(NB_X-1), id+NB_X, id+(NB_X+1) , id-(NB_X+1)};
	//shift (value to add to the actual Node's id) in our grid to obtain the id of the destination Node
	//in the good order : top, top right, right, bottom right, bottom, bottom left, left, top left


	//%%%%%%%%%%%%%%%%%%%% Do the initialization of the different edges %%%%%%%%%%%%%%%%%%%%%%%%
	direct_edges.reserve(MAX_NB_EDGES);

	//Edges pointing outside the map are attribuated a weight of 0 and an id destination of -1
	//We give the Edge class constructor the Edge's id, the pointed Node's id and the weight of the Edge (length)

	//%%%%%%%%%%%%%%%%%%%%%%%%  CHANGE FOR A SWITCH CASE AND OTHER FUNCTION (TOO BIG HERE)%%%%%%%%%%%%%%%%%%%%%%%%%
	if( id == 0 )//Node in the top left corner
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( i==0 || i>4 )//edges pointing outside the map
			{
				direct_edges.push_back (Edge (i, -1, 0));
			}
			else//edges pointing in the map
			{
				if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
				{
					direct_edges.push_back (Edge (i, shift_index[i], 1));
				}
				else//indice impair (the 4 diagonals)
				{
					direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
				}
			}
		}
	}
	else if( id < (NB_X - 1) )//Nodes at the top border (except corners) 
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( i<=1 || i==7 )//edges pointing outside the map
			{
				direct_edges.push_back (Edge (i, -1, 0));
			}
			else//edges pointing in the map
			{
				if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
				{
					direct_edges.push_back (Edge (i, shift_index[i], 1));
				}
				else//indice impair (the 4 diagonals)
				{
					direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
				}
			}
		}
	}
	else if( id == (NB_X - 1) )//Node at the top right corner
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( i<=4 || i==7 )//edges pointing outside the map
			{
				direct_edges.push_back (Edge (i, -1, 0));
			}
			else//edges pointing in the map
			{
				if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
				{
					direct_edges.push_back (Edge (i, shift_index[i], 1));
				}
				else//indice impair (the 4 diagonals)
				{
					direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
				}
			}
		}
	}
	else if( (id % (NB_X - 1)) == 0 )//Nodes at the right border (except corners)
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( i>=1 && i<=3 )//edges pointing outside the map
			{
				direct_edges.push_back (Edge (i, -1, 0));
			}
			else//edges pointing in the map
			{
				if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
				{
					direct_edges.push_back (Edge (i, shift_index[i], 1));
				}
				else//indice impair (the 4 diagonals)
				{
					direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
				}
			}
		}
	}
	else if( id == (NB_X*NB_Y - 1) )//Node at the bottom right corner
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( i>=1 && i<=5 )//edges pointing outside the map
			{
				direct_edges.push_back (Edge (i, -1, 0));
			}
			else//edges pointing in the map
			{
				if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
				{
					direct_edges.push_back (Edge (i, shift_index[i], 1));
				}
				else//indice impair (the 4 diagonals)
				{
					direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
				}
			}
		}
	}
	else if( id > ((NB_Y-1) * NB_X) )//Nodes at the bottom border (except corners)
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( i>=3 && i<=5 )//edges pointing outside the map
			{
				direct_edges.push_back (Edge (i, -1, 0));
			}
			else//edges pointing in the map
			{
				if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
				{
					direct_edges.push_back (Edge (i, shift_index[i], 1));
				}
				else//indice impair (the 4 diagonals)
				{
					direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
				}
			}
		}
	}
	else if( id == ((NB_Y-1) * NB_X) )//Node at the bottom left corner
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( i>=1 && i<=5 )//edges pointing outside the map
			{
				direct_edges.push_back (Edge (i, -1, 0));
			}
			else//edges pointing in the map
			{
				if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
				{
					direct_edges.push_back (Edge (i, shift_index[i], 1));
				}
				else//indice impair (the 4 diagonals)
				{
					direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
				}
			}
		}
	}
	else if( (id % NB_X) == 0 )//Nodes at the left border (except corners)
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( i>=5 )//edges pointing outside the map
			{
				direct_edges.push_back (Edge (i, -1, 0));
			}
			else//edges pointing in the map
			{
				if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
				{
					direct_edges.push_back (Edge (i, shift_index[i], 1));
				}
				else//indice impair (the 4 diagonals)
				{
					direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
				}
			}
		}
	}
	else//node not on the map border
	{
		for(int i = 0; i < MAX_NB_EDGES; i++)
		{
			if( (i % 2) == 0 )//indice pair (top, left, bottom, right)
			{
				direct_edges.push_back (Edge (i, shift_index[i], 1));
			}
			else//indice impair (the 4 diagonals)
			{
				direct_edges.push_back (Edge (i, shift_index[i], sqrt(2)));
			}
		}
	}
}
//%%%%%%%%%%%%%%%%%%%% End of the the initialization of the different edges %%%%%%%%%%%%%%%%%%%%%%%%


Node::~Node()
{}


void Node::node_set_previous_node_id(int id_prev)
{
	previous_node_id=id_prev;
}

void Node::node_set_distance_to_goal(float x_g, float y_g)
{
	distance_to_goal = sqrt( pow((x_g - coordinates[X]),2) + pow((y_g - coordinates[Y]),2) );
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


float Node::node_get_distance_to_goal()
{
	return distance_to_goal;
}










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

