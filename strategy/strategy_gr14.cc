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
	strat->target[6].y = -1.25;
	strat->target[6].status = TARGET_FREE;

	strat->target[7].x = -0.4;
	strat->target[7].y = -0.6;
	strat->target[7].status = TARGET_FREE;

	strat->start_base.x = +0.7;
	strat->start_base.y = +0.6;

	strat->target_base.x = -0.7;
	strat->target_base.y = -1.2;

	strat->current_target_id = 0;
	strat->current_point_id = 0;

	strat->last_t_wait = 0;
	strat->last_t_path = 0;

	strat->carrying_target_id[0] = -1;
	strat->carrying_target_id[1] = -1;

	strat->prev_nb_target_carrying = 0;

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
	// printf("Target id: %d\n", target_id);

	array<float, 2> source_pos;
	array<float, 2> goal_pos;

	switch (strat->main_state)
	{
		case GAME_STATE_COMPUTE_PATH:
			cvs->outputs->flag_release = 0;
			source_pos[0] = cvs->kalman_pos->x;
			source_pos[1] = cvs->kalman_pos->y;

			if (inputs->nb_targets < 2)
			{
				strat->status = STRAT_TARGET;
				int k = 0;
				while (strat->target[target_id].status == TARGET_WON)
				{
					if (++k > 8)
					{
						printf("END GAME at t = %.2f!\n", cvs->inputs->t);
						strat->main_state = GAME_STATE_E;
						return;
					}
					update_current_target_id(strat);
					target_id = strat->current_target_id;
				}
				goal_pos[0] = strat->target[target_id % 8].x;
				goal_pos[1] = strat->target[target_id % 8].y;
				// printf("Going from x: %.3f y: %.3f\n", source_pos[0], source_pos[1]);
				// printf("Going for target %d at x: %.3f y: %.3f\n", target_id+1, goal_pos[0], goal_pos[1]);

			}
			else
			{
				strat->status = STRAT_SCORING;
				goal_pos[0] = strat->target_base.x;
				goal_pos[1] = strat->target_base.y;
			}
			path = path_planning_compute(cvs, source_pos, goal_pos);
			if (!path.empty())//if we're not able to compute a path, we do it again until we got one
			{
				strat->main_state = GAME_STATE_GO_TO_GOAL;
				// printf("Size in follow_path: %lu\n", path.size());
				// for (size_t l = 0; l< path.size(); l++)
				// {
				// 	printf("Going for x:%.3f y: %.3f\n", path[l][0], path[l][1]);
				// }
			}
			else
			{
				printf("Path is empty\n");
			}
			strat->last_t_path = inputs->t;
			break;

		case GAME_STATE_GO_TO_GOAL:
			if (inputs->t - strat->last_t_path >= 0.3)
			{
			 	strat->main_state = GAME_STATE_COMPUTE_PATH;
				break;
			}
			follow_path(cvs, path);
			strat->last_t_wait = inputs->t;
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
			if ( inputs->t - strat->last_t_wait > 1.5)
 			{
 				strat->last_t_wait = inputs->t;
 				if (strat->prev_nb_target_carrying + 1 == inputs->nb_targets)
 				{
 					// Target picked up
 					strat->prev_nb_target_carrying++;
 					if (inputs->nb_targets == 1)
					{
						strat->carrying_target_id[0] = strat->current_target_id;
					}
					else
					{
						strat->carrying_target_id[1] = strat->current_target_id;
					}
					strat->current_target_id = (strat->current_target_id + 1)%8;
					strat->current_point_id = 0;
 				}
 				strat->main_state = GAME_STATE_COMPUTE_PATH;
 			}
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

void update_current_target_id(Strategy* strat)
{
	strat->current_target_id = (strat->current_target_id + 1) % 8;
	return;
}

NAMESPACE_CLOSE();
