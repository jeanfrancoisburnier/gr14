/*! 
 * \author Group 14
 * \file path_regulation_gr14.h
 * \brief regulation to follow a given path
 */

#ifndef _PATH_REGULATION_GR14_H_
#define _PATH_REGULATION_GR14_H_

#include "CtrlStruct_gr14.h"
#include <vector>
#include <array>

using namespace std;

NAMESPACE_INIT(ctrlGr14);


int get_actual_index_node_path();
void update_actual_index_node_path(int indice);


/*
 * \brief: 	follow a given path
 * \param: 	path 	path composed of the coordinate
 *					of all the intermediate point
 */
void follow_path(CtrlStruct *cvs, vector<array<float,2> > path);

/*
 * \brief: 	get the new speed command for the wheels
 * \param: 	gamma	direction of the point to reach in rad
 *			l_speed	point on left speed
 *			r_speed point on right speed
 */
void get_new_speed(float gamma, float *l_speed, float *r_speed);

void reset_current_point_id(void);

NAMESPACE_CLOSE();

#endif
