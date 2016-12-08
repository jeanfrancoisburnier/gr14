/*! 
 * \author Group 14
 * \file path_planning_gr14.h
 * \brief path-planning algorithm
 */



#ifndef _PATH_PLANNING_GR14_H_
#define _PATH_PLANNING_GR14_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"
#include "node_gr14.h"

#include <array>
#include <vector>

//#define NB_OPPONENTS 2
#define NB_FIXED_OBSTACLES 10 // 8 fixed obstacles + 2 oponents
#define ROBOT_SIZE 0.260 //[m] diameter
#define SECURITY_RANGE 0.020 //[m] security range add to the obstacles 
 							//(and to robot's size / 2) to be able to consider the robot as a material point

using namespace std; //to be able to use array

NAMESPACE_INIT(ctrlGr14);

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


vector<array<float,2> > path_planning_compute(CtrlStruct *cvs, array<float, 2> source_pos, array<float, 2> goal_pos);

void init_grid();
vector<array<float,2> > generate_path(int source_id,int goal_id);
void a_star(CtrlStruct *cvs, int source_id,int goal_id);
void free_path_planning(PathPlanning *path);

void reset_value_grid(vector<Obstacles> moving_obstacles);
array<Obstacles, NB_FIXED_OBSTACLES> initialization_fixed_obstacles();
vector<Obstacles> update_moving_obstacles(CtrlStruct *cvs);
void update_grid(CtrlStruct *cvs);

int search_free_neighbours(int id_occ);


NAMESPACE_CLOSE();

#endif
