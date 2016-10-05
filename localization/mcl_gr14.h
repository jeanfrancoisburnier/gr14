/*! 
 * \author Group 14
 * \file mcl_gr14.h
 * \brief localization using Monte Carlo technic
 */

#ifndef _MCL_GR14_H_
#define _MCL_GR14_H_

#include "CtrlStruct_gr14.h"
#include "init_pos_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/// Markov main structure
struct MCLStruct
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void mcl_update(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
