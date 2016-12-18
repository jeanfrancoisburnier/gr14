/*! 
 * \author Group 14
 * \file strategy_gr14.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_GR14_H_
#define _STRATEGY_GR14_H_

#define NB_TARGET 8 //number of target

#include "CtrlStruct_gr14.h"

#define NB_TARGET 8 //number of target

NAMESPACE_INIT(ctrlGr14);

/// 'main_state' states (adapt with your own states)
enum {GAME_STATE_INITIAL, GAME_STATE_COMPUTE_PATH, GAME_STATE_GO_TO_GOAL, GAME_STATE_D, GAME_STATE_E, GAME_STATE_WAIT, GAME_STATE_BLOCKED};

typedef enum Strategy_status
{

	STRAT_TARGET,
	STRAT_HIDDING,
	STRAT_RECOVERING,
	STRAT_SCORING

} strategy_status_t;

typedef enum Target_status
{

	TARGET_FREE,
	TARGET_STOLEN,
	TARGET_CARRIED,
	TARGET_WON

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
	strategy_status_t status;
	int carrying_target_id[2];
	int prev_nb_target_carrying;
	uint8_t current_target_id;
	uint8_t current_point_id;
	target_t target[NB_TARGET];
	start_base_t  start_base;
	target_base_t target_base;
	double last_t_wait;
	double last_t_path;
	int attempts;
	int remaining_targets;
	
} Strategy;

Strategy* init_strategy(CtrlStruct *cvs);
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);
void update_current_target_id(Strategy* strat);
void update_target_status(CtrlStruct *cvs);
void deblock_robot(CtrlStruct *cvs, bool orient);
NAMESPACE_CLOSE();

#endif
