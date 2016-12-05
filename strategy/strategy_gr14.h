/*! 
 * \author Group 14
 * \file strategy_gr14.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_GR14_H_
#define _STRATEGY_GR14_H_

#include "CtrlStruct_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/// 'main_state' states (adapt with your own states)
enum {GAME_STATE_INITIAL, GAME_STATE_COMPUTE_PATH, GAME_STATE_GO_TO_GOAL, GAME_STATE_D, GAME_STATE_E, GAME_STATE_WAIT};

typedef enum Target_status
{

	TARGET_FREE,
	TARGET_STOLEN,
	TARGET_CARRYING

} Target_status_t;

typedef struct Target
{

	double x;
	double y;
	Target_status_t status;

} target_t;

typedef struct Start_base
{

	double x;
	double y;
	bool targets;

} start_base_t;

typedef struct Target_base
{

	double x;
	double y;

} target_base_t;

/// strategy main structure
typedef struct Strategy
{
	int main_state; ///< main state of the strategy
	uint8_t current_target_id;
	target_t target[8];
	start_base_t  start_base;
	target_base_t target_base;
	
} Strategy;

Strategy* init_strategy();
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
