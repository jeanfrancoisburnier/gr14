/*! 
 * \author Group 14
 * \file path_planning_gr14.h
 * \brief path-planning algorithm
 */



#ifndef _PATH_PLANNING_GR14_H_
#define _PATH_PLANNING_GR14_H_ 
 


#include "node_gr14.h"

#include <array>
#include <vector>

#define NB_OBSTACLES 10 // 8 fixed obstacles + 2 oponents

using namespace std; //to be able to use array


/// path-planning main structure
struct PathPlanning
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};



//rectangle delimited by two extreme corner (first : top left, second : bottom right) 
struct Obstacles 
{
	array<float, 2> first_corner;
	array<float, 2> second_corner;
};



void init_grid();
vector<array<float,2> > generate_path(int source_id,int goal_id);
void a_star(int source_id,int goal_id);
void free_path_planning(PathPlanning *path);
void reset_visited_value();
array<Obstacles, NB_OBSTACLES> initialization_obstacles();


#endif
