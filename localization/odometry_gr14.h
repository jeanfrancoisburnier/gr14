/*! 
 * \author Group 14
 * \file odometry_gr14.h
 * \brief odometry of the robot
 */

#ifndef _ODOMETRY_GR14_H_
#define _ODOMETRY_GR14_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"

NAMESPACE_INIT(ctrlGr14);

void update_odometry(CtrlStruct *cvs);
double wheel_speed_meter(double wheel_speed_rad,double wheel_radius);

NAMESPACE_CLOSE();

#endif
