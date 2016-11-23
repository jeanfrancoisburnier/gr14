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

typedef struct Traget_path
{

	float x;
	float y;

} target_path_t;

typedef struct  Target 
{
	int nb_via;
	target_path_t* target_path;

} target_t;

typedef struct Path
{
	int nb_target;
	int current_target;
	target_t* targets;

	double last_t;

} path_t;

void follow_path(CtrlStruct *cvs, vector<array<float,2> > path);
// void follow_path(CtrlStruct *cvs, path_t* tab);
void reach_single_target(float gamma, float *l_speed, float *r_speed);

NAMESPACE_CLOSE();

#endif
