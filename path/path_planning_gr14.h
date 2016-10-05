/*! 
 * \author Group 14
 * \file path_planning_gr14.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR14_H_
#define _PATH_PLANNING_GR14_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/// path-planning main structure
struct PathPlanning
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
