/*! 
 * \author Group 14
 * \file kalman_gr14.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR14_H_
#define _KALMAN_GR14_H_

#include "CtrlStruct_gr14.h"
#include "init_pos_gr14.h"
#include "matrice.h"


NAMESPACE_INIT(ctrlGr14);

/// Kalman main structure
typedef struct KalmanStruc
{
	double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]

	double last_t; ///< last time position was updated 
	double P_k[3][3]; // The error covariance matrix
} KalmanStruc;

void kalman(CtrlStruct *cvs);


NAMESPACE_CLOSE();

#endif
