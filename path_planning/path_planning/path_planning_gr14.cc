#include "path_planning_gr14.h"
#include <math.h>
#include <queue>


#include "node_gr14.h"
#include <fstream>




const int NB_NODES = NB_X * NB_Y;

using namespace std; //to be able to use array


/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */

static vector<Node> nodes_grid; //creation of our Node's grid in a global way, so each function of path_planning_gr14.cc will be able to use it



void init_grid()
{


	// ----- path-planning initialization start ----- //

	// ------ Initialization of the obstacles coordinates ------ //
	array<Obstacles, NB_OBSTACLES> list_obst;
	list_obst = initialization_obstacles();

	float x_node = 0.0;
	float y_node = 0.0;

	//static bool fini_grid = false;//to delete when note printing

	// ----- Creation of our Node's grid ----- //
	nodes_grid.reserve(NB_NODES);//The size will not change anymore after this loop

	for(int id_n=0; id_n<NB_NODES; id_n++)
	{
		y_node = peak_y - (id_n/NB_X) * square_length - square_length/2;
		//warning to understand this formula be cautious that id_n/NB_X is an operation between (int)
		
		x_node = -peak_x + square_length/2 + (id_n%NB_X) * square_length;
		//warning to understand this formula be cautious that id_n%NB_X is an operation between (int)

		bool state_pos = FREE;
		for(int j=0; j<NB_OBSTACLES; j++)
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
		if(state_pos == FREE)
		{
			nodes_grid.push_back( Node (id_n, FREE, x_node, y_node));
		}
		else
		{
			nodes_grid.push_back( Node (id_n, OCCUPIED, x_node, y_node));
		}


		
		/*if(fini_grid == false)
		{
			printf("X = %.3f, Y = %.3f, %s, id = %d, t %d 	 tr %d 	 r %d	br %d  	b %d	bl %d	l %d	tl %d\n",
				x_node, y_node, nodes_grid[id_n].node_get_free_position()? "FREE":"OCCUPIED", id_n,
				nodes_grid[id_n].node_get_edges()[0].edge_get_id_connected_node(), nodes_grid[id_n].node_get_edges()[1].edge_get_id_connected_node(),
				nodes_grid[id_n].node_get_edges()[2].edge_get_id_connected_node(), nodes_grid[id_n].node_get_edges()[3].edge_get_id_connected_node(),
				nodes_grid[id_n].node_get_edges()[4].edge_get_id_connected_node(), nodes_grid[id_n].node_get_edges()[5].edge_get_id_connected_node(),
				nodes_grid[id_n].node_get_edges()[6].edge_get_id_connected_node(), nodes_grid[id_n].node_get_edges()[7].edge_get_id_connected_node());

			set_output(x_node, "x_pos");
			set_output(y_node, "y_pos");
			set_output(nodes_grid[id_n].node_get_free_position(), "etat_node");
		}*/
	}
	//fini_grid = true;

	// ----- end of the creation of the Node's grid ----- //

	// ----- path-planning initialization end ----- //

	

	// return
    const char *path_file="/Users/antoinelaurens/Desktop/path_planning/path_planning/coordinates.txt";
    ofstream myfile(path_file);
    for (int i = 0;i<nodes_grid.size();i++)
    {
        array<float,2> coord =nodes_grid[i].node_get_coordinates();
        double x_coord = coord[X];
        double y_coord = coord[Y];
        
        myfile << "Node " << i << " x:" << x_coord << " y:" << y_coord << endl;
    }
	return;
}




/*! \brief do the a-star algorithm to find the optimal path
 * \param[in]  source node
 * \param[in]  goal node
 * \param[in,out] node_grid that we modify throughout the function
 */
void a_star(int source_id, int goal_id)
{
    if (source_id >= nodes_grid.size() || source_id < 0 || goal_id >= nodes_grid.size() || goal_id < 0
        || !nodes_grid[source_id].node_get_free_position() || !nodes_grid[goal_id].node_get_free_position())
    {
        printf("invalid start or goal\n");
        return;
    }
    //reset the values of the boolean visited to false
    reset_visited_value();
    
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
            printf("Unable to find path\n");
            return;
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
    
    if (source_id >= nodes_grid.size() || source_id < 0 || goal_id >= nodes_grid.size() || goal_id < 0
        || !nodes_grid[source_id].node_get_free_position() || !nodes_grid[goal_id].node_get_free_position())
    {
        printf("invalid start or goal\n");
        return path;
    }
    
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








void reset_visited_value()
{
	for(int i=0; i<NB_NODES; i++)
	{
		nodes_grid[i].node_set_visited(false);
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

