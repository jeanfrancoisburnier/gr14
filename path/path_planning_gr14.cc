#include "path_planning_gr14.h"
#include "init_pos_gr14.h"
#include "opp_pos_gr14.h"
#include "useful_gr14.h"
#include <math.h>

#include "node_gr14.h"

#include <vector>
#include <array>

#define FREE true		//indicates if a node is free
#define OCCUPIED false	//indicates if a node is occupied

const int NB_NODES = NB_X * NB_Y;

using namespace std; //to be able to use array

NAMESPACE_INIT(ctrlGr14);

/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */

static vector<Node> nodes_grid; //creation of our Node's grid in a global way, so each function of path_planning_gr14.cc will be able to use it



void init_grid()
{
	PathPlanning *path;

	// memory allocation
	path = (PathPlanning*) malloc(sizeof(PathPlanning));

	// ----- path-planning initialization start ----- //

	// ------ Initialization of the obstacles coordinates ------ //
	array<Obstacles, NB_OBSTACLES> list_obst;
	list_obst = initialization_obstacles();

	float x_node = 0.0;
	float y_node = 0.0;


	// ----- Creation of our Node's grid ----- //
	nodes_grid.reserve(NB_NODES);//The size will not change anymore after this loop

	for(int id_n=0; id_n<NB_NODES; id_n++)
	{
		y_node = peak_y - (id_n/NB_X) * square_length - square_length/2;
		//warning to understand this formula be cautious that id_n/NB_X is an operation between (int)
		
		x_node = -peak_x + square_length/2 + (id_n%NB_X) * square_length;
		//warning to understand this formula be cautious that id_n%NB_X is an operation between (int)

		for(int j=0; j<NB_OBSTACLES; j++)
		{
			if( (x_node > list_obst[j].first_corner[X]) && (x_node < list_obst[j].second_corner[X]) 
				&& (y_node < list_obst[j].first_corner[Y]) && (y_node > list_obst[j].second_corner[Y]) ) //if the Node is on an obstacle --> occupied
			{
				nodes_grid.push_back( Node (id_n, OCCUPIED, x_node, y_node));
			}
			else//if the Node is not on an obstacle --> Free
			{
				nodes_grid.push_back( Node (id_n, FREE, x_node, y_node));
			}
		}
	}
	// ----- end of the creation of the Node's grid ----- //

	// ----- path-planning initialization end ----- //

	

	// return 
	return;
}




/*! \brief do the a-star algorithm to find the optimal path
 * \param[in]  source node
 * \param[in]  goal node
 * \param[in,out] node_grid that we modify throughout the function
 */
void a_star(node Source,node Goal)
{
	vector<node> open_paths;
	vector<int> modified ids;
	node next;

	if (source = goal)
	{
		return;
	}

	
}

// generate a vector of x y coordinates to follow
vector<array<float,2>> generate_path(node Source, node Goal)
{

}



/*! \brief close the path-planning algorithm (memory released)
 * 
 * \param[in,out] path path-planning main structure
 */
void free_path_planning(PathPlanning *path)
{
	// ----- path-planning memory release start ----- //


	// ----- path-planning memory release end ----- //

	free(path);
}



void reset_heuristic_value()
{
	for(int i=0; i<NB_NODES; i++)
	{
		nodes_grid[i].node_set_heuristic_value(H_VALUE_INIT);
	}
}




array<Obstacles, NB_OBSTACLES> initialization_obstacles()
{
	array<Obstacles, NB_OBSTACLES> list_obstacles;
	list_obstacles[0].first_corner[X] = -0.850;
	list_obstacles[0].first_corner[Y] = 1.0;
	list_obstacles[0].second_corner[X] = -0.350;
	list_obstacles[0].second_corner[Y] = 0.680;

	list_obstacles[1].first_corner[X] = 0.330;
	list_obstacles[1].first_corner[Y] = 1.350;
	list_obstacles[1].second_corner[X] = 0.650;
	list_obstacles[1].second_corner[Y] = 0.850;

	list_obstacles[2].first_corner[X] = 0.330;
	list_obstacles[2].first_corner[Y] = -0.850;
	list_obstacles[2].second_corner[X] = 0.650;
	list_obstacles[2].second_corner[Y] = -1.350;

	list_obstacles[3].first_corner[X] = -0.850;
	list_obstacles[3].first_corner[Y] = -0.680;
	list_obstacles[3].second_corner[X] = -0.350;
	list_obstacles[3].second_corner[Y] = -1.0;

	list_obstacles[4].first_corner[X] = -0.350;
	list_obstacles[4].first_corner[Y] = 0.550;
	list_obstacles[4].second_corner[X] = 0.350;
	list_obstacles[4].second_corner[Y] = 0.150;

	list_obstacles[5].first_corner[X] = -0.350;
	list_obstacles[5].first_corner[Y] = 0.150;
	list_obstacles[5].second_corner[X] = 0.050;
	list_obstacles[5].second_corner[Y] = -0.150;

	list_obstacles[6].first_corner[X] = -0.650;
	list_obstacles[6].first_corner[Y] = 0.250;
	list_obstacles[6].second_corner[X] = -0.350;
	list_obstacles[6].second_corner[Y] = -0.250;

	list_obstacles[7].first_corner[X] = -0.350;
	list_obstacles[7].first_corner[Y] = -0.150;
	list_obstacles[7].second_corner[X] = 0.350;
	list_obstacles[7].second_corner[Y] = -0.550;


	//oponents (do not have any value for now --> would be great to initialize with the value computed with the lazer tower)
	list_obstacles[8].first_corner[X] = 0.00;
	list_obstacles[8].first_corner[Y] = 0.00;
	list_obstacles[8].second_corner[X] = 0.00;
	list_obstacles[8].second_corner[Y] = 0.00;

	list_obstacles[9].first_corner[X] = 0.00;
	list_obstacles[9].first_corner[Y] = 0.00;
	list_obstacles[9].second_corner[X] = 0.00;
	list_obstacles[9].second_corner[Y] = 0.00;

	return list_obstacles;
}


NAMESPACE_CLOSE();
