#include "path_planning_gr14.h"
#include "init_pos_gr14.h"
#include "opp_pos_gr14.h"
#include "useful_gr14.h"
#include <math.h>

#define NB_OBSTACLES 10 // 8 fixes obstacles + 2 oponents

NAMESPACE_INIT(ctrlGr14);

/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */
PathPlanning* init_path_planning()
{
	PathPlanning *path;

	// memory allocation
	path = (PathPlanning*) malloc(sizeof(PathPlanning));

	// ----- path-planning initialization start ----- //

	//creation of the obstacles
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


	

	// ----- path-planning initialization end ----- //

	// return structure initialized
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

NAMESPACE_CLOSE();
