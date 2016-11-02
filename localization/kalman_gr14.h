/*! 
 * \author Group 14
 * \file kalman_gr14.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR14_H_
#define _KALMAN_GR14_H_

#include "namespace_ctrl.h"
#include "CtrlStruct_gr14.h"
#include "matrice_gr14.h"


NAMESPACE_INIT(ctrlGr14);

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
