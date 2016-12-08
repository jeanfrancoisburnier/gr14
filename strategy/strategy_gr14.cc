#include "strategy_gr14.h"
#include "path_planning_gr14.h"
#include "speed_regulation_gr14.h"
#include "path_regulation_gr14.h"
#include "path_planning_gr14.h"
#include "init_pos_gr14.h"
#include "opp_pos_gr14.h"
#include "odometry_gr14.h"
#include "node_gr14.h"
#include <vector>
#include <array>
#include <math.h>

using namespace std;

NAMESPACE_INIT(ctrlGr14);

vector<array<float,2> > path;

/*! \brief intitialize the strategy structure
 * 
 * \return strategy structure initialized
 */
Strategy* init_strategy()
{
	Strategy *strat;

	strat = (Strategy*) malloc(sizeof(Strategy));

	strat->target[0].x = -0.8;
	strat->target[0].y = +0.0;
	strat->target[0].status = TARGET_FREE;

	strat->target[1].x = -0.4;
	strat->target[1].y = +0.6;
	strat->target[1].status = TARGET_FREE;

	strat->target[2].x = +0.7;
	strat->target[2].y = +0.6;
	strat->target[2].status = TARGET_FREE;

	strat->target[3].x = +0.25;
	strat->target[3].y = +1.25;
	strat->target[3].status = TARGET_FREE;

	strat->target[4].x = +0.1;
	strat->target[4].y = +0.0;
	strat->target[4].status = TARGET_FREE;

	strat->target[5].x = +0.7;
	strat->target[5].y = -0.6;
	strat->target[5].status = TARGET_FREE;

	strat->target[6].x = +0.25;
	strat->target[6].y = +1.25;
	strat->target[6].status = TARGET_FREE;

	strat->target[7].x = +0.25;
	strat->target[7].y = -1.25;
	strat->target[7].status = TARGET_FREE;

	strat->start_base.x = +0.7;
	strat->start_base.y = +0.6;

	strat->target_base.x = -0.7;
	strat->target_base.y = -1.2;

	strat->current_target_id = 0;

	return strat;
}

/*! \brief release the strategy structure (memory released)
 * 
 * \param[out] strat strategy structure to release
 */
void free_strategy(Strategy *strat)
{
	free(strat);
}

/*! \brief startegy during the game
 * 
 * \param[in,out] cvs controller main structure
 */
void main_strategy(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	CtrlIn *inputs;
	uint8_t target_id;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;
	target_id = strat->current_target_id;


	static double last_t = inputs->t;

	array<float, 2> source_pos;
	array<float, 2> goal_pos;

	switch (strat->main_state)
	{
		case GAME_STATE_INITIAL:
			source_pos[0] = /*0.7;//*/cvs->kalman_pos->x;
			source_pos[1] = /*1.0;//*/cvs->kalman_pos->y;
			//goal_pos[0] = strat->target[0].x;
			//goal_pos[1] = strat->target[0].y;
			goal_pos[0] = 0.400;
			goal_pos[1] = -1.0;

			path = path_planning_compute(cvs, source_pos, goal_pos);
			strat->main_state = GAME_STATE_GO_TO_GOAL;
			break;

		case GAME_STATE_COMPUTE_PATH:
			source_pos[0] = cvs->kalman_pos->x;
			source_pos[1] = cvs->kalman_pos->y;
			if (inputs->nb_targets < 2)
			{
				while (target_id < 7 && strat->target[target_id].status == TARGET_STOLEN)
				{
					target_id++;
				}
				goal_pos[0] = strat->target[target_id].x;
				goal_pos[1] = strat->target[target_id].y;
			}
			else
			{
				goal_pos[0] = strat->target_base.x;
				goal_pos[1] = strat->target_base.y;
			}
			path = path_planning_compute(cvs, source_pos, goal_pos);
			strat->main_state = GAME_STATE_GO_TO_GOAL;

			// printf("Size in follow_path: %lu\n", path.size());
			// for (size_t l = 0; l< path.size(); l++)
 			// 	{
 			// 		printf("Going for x:%.3f y: %.3f\n", path[l][0], path[l][1]);
 			// 	}
			break;

		case GAME_STATE_GO_TO_GOAL:
			follow_path(cvs, path);
			last_t = inputs->t;
			break;

		case GAME_STATE_D:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_E:
			cvs->main_state = STOP_END_STATE;
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_WAIT:
			speed_regulation(cvs, 0.0, 0.0);
			if ( inputs->t - last_t > 1.5)
 			{
 				last_t = inputs->t;
 				strat->main_state = GAME_STATE_COMPUTE_PATH;
 			}
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
