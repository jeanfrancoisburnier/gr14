#include "path_planning_gr14.h"
#include "init_pos_gr14.h"
#include "opp_pos_gr14.h"
#include "useful_gr14.h"
#include <math.h>
#include <queue>
#include "set_output.h"

#include "node_gr14.h"



const int NB_NODES = NB_X * NB_Y;

using namespace std; //to be able to use array

NAMESPACE_INIT(ctrlGr14);

/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */

static vector<Node> nodes_grid; //creation of our Node's grid in a global way, so each function of path_planning_gr14.cc will be able to use it
static vector<int> id_occupied; //id of the Nodes who are on a fixed obstacle !


void init_grid()
{
	PathPlanning *path;

	// memory allocation
	path = (PathPlanning*) malloc(sizeof(PathPlanning));

	// ----- path-planning initialization start ----- //

	// ------ Initialization of the obstacles coordinates ------ //
	array<Obstacles, NB_FIXED_OBSTACLES> list_obst;
	list_obst = initialization_fixed_obstacles();

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

		bool state_pos = FREE;
		for(int j=0; j<NB_FIXED_OBSTACLES; j++)
		{
			if( (x_node > list_obst[j].first_corner[X]) && (x_node < list_obst[j].second_corner[X]) 
				&& (y_node < list_obst[j].first_corner[Y]) && (y_node > list_obst[j].second_corner[Y]) ) //if the Node is on an obstacle --> occupied
			{
				state_pos = OCCUPIED;
				id_occupied.push_back(id_n); //add the id of the node on an obstacle
				break;
			}
			else//if the Node is not on an obstacle --> Free
			{
				state_pos = FREE;
			}
		}

		nodes_grid.push_back( Node (id_n, state_pos, x_node, y_node));
	}
	// ----- end of the creation of the Node's grid ----- //
}







vector<array<float,2> > path_planning_compute(CtrlStruct *cvs, array<float, 2> source_pos, array<float, 2> goal_pos)
{
	int source_id = node_find_closest_node(source_pos[X], source_pos[Y]);
	printf("\nXs_return : %3f, Ys_return : %3f\n", nodes_grid[source_id].node_get_coordinates()[X], nodes_grid[source_id].node_get_coordinates()[Y]);
	printf("SOURCE_ID: %d\n", source_id);
    int goal_id = node_find_closest_node(goal_pos[X], goal_pos[Y]);
    printf("Xs_return : %3f, Ys_return : %3f\n", nodes_grid[goal_id].node_get_coordinates()[X], nodes_grid[goal_id].node_get_coordinates()[Y]);
	printf("goal_ID: %d\n", goal_id);


	if( source_id >= nodes_grid.size() || source_id < 0 || goal_id >= nodes_grid.size() || goal_id < 0 )
    {
        printf("invalid start or goal, outside the map\n");
        exit(EXIT_FAILURE);
        //Set the flag "path generated" to 0 and return Null
    }
    
    if( !nodes_grid[source_id].node_get_free_position() )
    {
    	printf("invalid start, on an occupied node\n");
    	source_id = search_free_neighbours(source_id);
    }
    
    if( !nodes_grid[goal_id].node_get_free_position() )
    {
    	printf("invalid goal, on an occupied node\n");
    	goal_id = search_free_neighbours(goal_id);
    }
  
    update_grid(cvs);
    a_star(cvs, source_id, goal_id);
    vector<array<float,2> > path = generate_path(source_id, goal_id);

    //Set the flag "path generated" to 1 and return path
    return path;
}




/*! \brief do the a-star algorithm to find the optimal path
 * \param[in]  source node
 * \param[in]  goal node
 * \param[in,out] nodes_grid that we modify throughout the function
 */
void a_star(CtrlStruct *cvs, int source_id, int goal_id)
{
    //initialize the values of the source node
    nodes_grid[source_id].node_set_distance_to_goal(nodes_grid[goal_id].node_get_coordinates());
    nodes_grid[source_id].node_set_distance_to_start(0);
    nodes_grid[source_id].node_set_heuristic_value(nodes_grid[source_id].node_get_distance_to_goal());
    nodes_grid[source_id].node_set_visited(true);
    
    //priority queue to sort nodes depending on the value of the heuristic function
    priority_queue<Node,vector<Node>,compare_heuristic > open_paths;
    
    // list of noe id that are reached by the current node and have not been visited yet
    // intermediate node
    vector<int> modified_ids;
    Node next = nodes_grid[source_id];
    
    //check if source and goal node are different
    if (nodes_grid[source_id].node_get_id() == nodes_grid[goal_id].node_get_id())
    {
    	printf("Already on goal\n");
        return;
    }
    
    // do algorithm until goal node has been reached
    while (next.node_get_id() != nodes_grid[goal_id].node_get_id())
    {
        //find the ids of the nodes that have not been visited yet
        modified_ids = next.scan_edges(nodes_grid,nodes_grid[goal_id]);
        
        //add to the queue the new nodes
        for(auto id:modified_ids)
        {
            open_paths.push(nodes_grid[id]);
        }
        
        //check if there are new nodes available in the queue and if not return an error
        if(open_paths.size() == 0)
        {
            printf("No nodes available in the queue\n");
        	exit(EXIT_FAILURE);
        }
        
        //take the next node with the lowest heuristic function and remove that element for the list
        next = open_paths.top();
        //printf("%d\n",next.node_get_id());
        open_paths.pop();
        
        //printf("node id: %d\t heuristic %f\n",next.node_get_id(),next.node_get_heuristic_value());
    }
    return;
    
}




// generate a vector of x y coordinates to follow
// the beginning of the vector is the source
vector<array<float,2> > generate_path(int source_id, int goal_id)
{
    vector<array<float,2> > path ;
        
    int next_id = goal_id;
    
    // insert the goal in the vector
    path.insert(path.begin(),nodes_grid[goal_id].node_get_coordinates());
    
    //add all the nodes coordinates of the path previously computed with a* using the previous_id each time 
    while (next_id != source_id)
    {
        next_id = nodes_grid[next_id].node_get_previous_node_id();
        path.insert(path.begin(),nodes_grid[next_id].node_get_coordinates());
    }
    return path;
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



//This function return the id of a free node close to the occupied one 
//We first search for direct neighbours but if they are also occupied we extend
//serach each time for direct neighbours even if there is some redundancy ... 
//(we can do an expension of 3 squares)
int search_free_neighbours(int id_occ)
{
	int id_fr;
	int id_test;
	vector<int> path_non_blocked;
	vector<Edge> edges_node_occ = nodes_grid[id_occ].node_get_edges();
	for(int i=0; i<MAX_NB_EDGES; i++)//first level
	{
		if( edges_node_occ[i].edge_get_weight() != 0 )//if we do not search for neighbours outside the map
		{
			id_test = edges_node_occ[i].edge_get_id_connected_node();

			path_non_blocked.push_back(id_test);

			if( nodes_grid[id_test].node_get_free_position() == FREE )//if the neighbor is free
			{
				id_fr = id_test;
				return id_fr;
			}
		}
	}
	int start_size = path_non_blocked.size();
	for(int j=0; j<start_size; j++)//second level
	{
		edges_node_occ = nodes_grid[ path_non_blocked[j] ].node_get_edges();
		//we take the edges of a node we want to inspect its surrondings

		for(int k=0; k<MAX_NB_EDGES; k++)
		{
			if( edges_node_occ[k].edge_get_weight() != 0 )//if we do not search for neighbours outside the map
			{
				id_test = edges_node_occ[k].edge_get_id_connected_node();

				path_non_blocked.push_back(id_test); 
				

				if( nodes_grid[id_test].node_get_free_position() == FREE )//if the neighbor is free
				{
					id_fr = id_test;
					return id_fr;
				}
			}
		}
	}

	for(int j=0; j<(path_non_blocked.size() - start_size); j++)//third level //maybe change the condition in the way we decide to solve the problem 1
	{
		edges_node_occ = nodes_grid[ path_non_blocked[j + start_size -1] ].node_get_edges();
		//we take the edges of a node we want to inspect its surrondings

		for(int k=0; k<MAX_NB_EDGES; k++)
		{
			if( edges_node_occ[k].edge_get_weight() != 0 )//if we do not search for neighbours outside the map
			{
				id_test = edges_node_occ[k].edge_get_id_connected_node();

				if( nodes_grid[id_test].node_get_free_position() == FREE )//if the neighbor is free
				{
					id_fr = id_test;
					return id_fr;
				}
			}
		}
	}
	return id_occ;//to change, if I don't find direct free neighbours --> expend out research.
}
/*MAybe do a paramtrization of the max squares in an obstacle and do a research on a number of a parametrized  levels
We can do the algo above a certain number of time (stop condition depending on the max sqqaures in an obstacles (node)*/


//update the grid when we start a new path planning
void update_grid(CtrlStruct *cvs)
{
	array<Obstacles, NB_OPPONENTS> mov_obstacles;
	mov_obstacles = update_moving_obstacles(cvs);

	reset_value_grid(mov_obstacles);
}



//reset visited_value and update the free_positions of the nodes where there is now opponents
void reset_value_grid(array<Obstacles, NB_OPPONENTS> moving_obstacles)
{
	for(int i=0; i<NB_NODES; i++)
	{
		for(int k=0; k<id_occupied.size(); k++)
		{
			if( i!= (id_occupied[k]) )//if we're not already on a fixed obstacle
			{
				nodes_grid[i].node_set_visited(false);

				bool state_pos;
				float x_node = 0.0;
				float y_node = 0.0;

				array<float, 2> pos_node = nodes_grid[i].node_get_coordinates();
				x_node = pos_node[X];
				y_node = pos_node[Y];

				for(int j=0; j<NB_OPPONENTS; j++)
				{
					if( (x_node > moving_obstacles[j].first_corner[X]) && (x_node < moving_obstacles[j].second_corner[X]) 
						&& (y_node < moving_obstacles[j].first_corner[Y]) && (y_node > moving_obstacles[j].second_corner[Y]) )
						//if the Node is on an opponent --> occupied
					{
						state_pos = OCCUPIED;
						break;
					}
					else//if the Node is not on an obstacle --> Free
					{
						state_pos = FREE;
					}
				}

				nodes_grid[i].node_set_free_position(state_pos);
			}
			else // if on fixed obstacle
			{
				nodes_grid[i].node_set_free_position(OCCUPIED);
				break;
			}
		}
	}
}



array<Obstacles, NB_OPPONENTS> update_moving_obstacles(CtrlStruct *cvs)
{
	array<Obstacles, NB_OPPONENTS> moving_obstacles;
 
	moving_obstacles[0].first_corner[X] = cvs->opp_pos->x[0] - ROBOT_SIZE/2 - SECURITY_RANGE;
	moving_obstacles[0].first_corner[Y] =  cvs->opp_pos->y[0] + ROBOT_SIZE/2 + SECURITY_RANGE;
	moving_obstacles[0].second_corner[X] =  cvs->opp_pos->x[0] + ROBOT_SIZE/2 + SECURITY_RANGE;
	moving_obstacles[0].second_corner[Y] =  cvs->opp_pos->y[0] - ROBOT_SIZE/2 - SECURITY_RANGE; 

	moving_obstacles[1].first_corner[X] =  cvs->opp_pos->x[1] - ROBOT_SIZE/2 - SECURITY_RANGE;
	moving_obstacles[1].first_corner[Y] =  cvs->opp_pos->y[1] + ROBOT_SIZE/2 + SECURITY_RANGE;
	moving_obstacles[1].second_corner[X] =  cvs->opp_pos->x[1] + ROBOT_SIZE/2 + SECURITY_RANGE;
	moving_obstacles[1].second_corner[Y] =  cvs->opp_pos->y[1] - ROBOT_SIZE/2 - SECURITY_RANGE; 

	return moving_obstacles;
}



array<Obstacles, NB_FIXED_OBSTACLES> initialization_fixed_obstacles()
{
	array<Obstacles, NB_FIXED_OBSTACLES> list_obstacles;
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

	return list_obstacles;
}



NAMESPACE_CLOSE();
