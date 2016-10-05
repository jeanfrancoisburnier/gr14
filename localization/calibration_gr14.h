/*! 
 * \author Group 14
 * \file calibration_gr14.h
 * \brief calibration of the robot
 */

#ifndef _CALIBRATION_GR14_H_
#define _CALIBRATION_GR14_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time for flag

	int flag; ///< flag for calibration

} RobotCalibration;

void calibration(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
