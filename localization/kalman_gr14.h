/*! 
 * \author Group 14
 * \file kalman_gr14.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR14_H_
#define _KALMAN_GR14_H_

#include "CtrlStruct_gr14.h"
#include "init_pos_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/// Kalman main structure
struct KalmanStruc
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
