/*! 
 * \author Group 14
 * \file node.h
 * \Node class
 */
#include "node_gr14.h"
#include "useful_gr14.h"
#include "math.h"

//Number of nodes along the x/y axis
#define NB_X 20 //define test (a definir avec la taille des cases et la taille de la map)
#define NB_Y 30


NAMESPACE_INIT(ctrlGr14);


Node::Node(int id_node, bool free_init): id(id_node), free_position(free_init)
{		
	//coordinates[X] = calcul with the id_repartition and the number of squares
	//coordinates[Y] = calcul with the id_repartition and the number of squares
	
	
	array<int, MAX_NB_EDGES> shift_index = {id-NB_X, id-(NB_X -1), id-1, id+1, id+(NB_X-1), id+NB_X, id+(NB_X+1) , id-(NB_X+1)};
	//shift (value to add to the actual Node's id) in our grid to obtain the id of the destination Node
	//in the good order : top, top right, right, bottom right, bottom, bottom left, left, top left


	//%%%%%%%%%%%%%%%%%%%% Do the initialization of the different edges %%%%%%%%%%%%%%%%%%%%%%%%\\
	direct_edges.reserve(MAX_NB_EDGES);

	//Edges pointing outside the map are attribuated a weight of 0 and an id destination of -1
	//We give the Edge class constructor the Edge's id, the pointed Node's id and the weight of the Edge (length)

	//%%%%%%%%%%%%%%%%%%%%%%%%TO CHANGE FOR A SWITCH CASE AND OTHER FUNCTION (TOO BIG HERE)%%%%%%%%%%%%%%%%%%%%%%%%%\\
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
	else if( (id > ((NB_Y-1) * NB_X)) && (id < NB_X*NB_Y-1) )//Nodes at the bottom border (except corners)
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
	//%%%%%%%%%%%%%%%%%%%% End of the the initialization of the different edges %%%%%%%%%%%%%%%%%%%%%%%%\\

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

