/*! 
 * \author Group 14
 * \file path_regulation_gr14.h
 * \brief regulation to follow a given path
 */

#ifndef _PATH_REGULATION_GR14_H_
#define _PATH_REGULATION_GR14_H_

#include "CtrlStruct_gr14.h"

NAMESPACE_INIT(ctrlGr14);

typedef struct Target_coordinate
{
	
	float x;
	float y;

} target_coordinate_t;

// void follow_path(CtrlStruct *cvs);
void follow_path(CtrlStruct *cvs, target_coordinate_t* tab, uint8_t size);
void reach_single_target(float gamma, float *l_speed, float *r_speed);

NAMESPACE_CLOSE();

#endif
