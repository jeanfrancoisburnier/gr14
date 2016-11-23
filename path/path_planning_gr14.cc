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
    int goal_id = node_find_closest_node(goal_pos[X], goal_pos[Y]);
    printf("SOURCE_ID: %d \t GOAL_ID: %d\n", source_id, goal_id);


	if (source_id >= nodes_grid.size() || source_id < 0 || goal_id >= nodes_grid.size() || goal_id < 0
        || !nodes_grid[source_id].node_get_free_position() || !nodes_grid[goal_id].node_get_free_position())
    {
        printf("invalid start or goal OCCUPIED or outside the map\n");
        exit(EXIT_FAILURE);
        //Set the flag "path generated" to 0 and return Null
    }
    //maybe add later a test if the goal is set on an obstacle

    update_grid(cvs);
    a_star(cvs, source_id, goal_id);
    vector<array<float,2> > path = generate_path(source_id, goal_id);

    //Set the flag "path generated" to 1 and return path
    return path;
}




/*! \brief do the a-star algorithm to find the optimal path
 * \param[in]  source node
 * \param[in]  goal node
 * \param[in,out] node_grid that we modify throughout the function
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
				//if the Node is on an obstacle --> occupied
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
