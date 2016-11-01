/*! 
 * \author Group 14
 * \file init_pos_gr14.h
 * \brief initial position of the robot
 */

#ifndef _INIT_POS_GR14_H_
#define _INIT_POS_GR14_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/// robot position
typedef struct RobotPosition
{
	double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]

	double last_t; ///< last time position was updated

} RobotPosition;

/// Kalman main structure
typedef struct KalmanStruct
{
	double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]

	double last_t; ///< last time position was updated 
	double P_k[3][3]; // The error covariance matrix
} KalmanStruct;

void set_init_position(int robot_id, RobotPosition *rob_pos);
void set_init_position_kalman(int robot_id, KalmanStruct *kalman_pos);

NAMESPACE_CLOSE();

#endif
