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

	// path.current_target = 0;

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

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;

	array<float, 2> source_pos;
	array<float, 2> goal_pos;

	switch (strat->main_state)
	{
		case GAME_STATE_A:
			follow_path(cvs, path);
			break;

		case GAME_STATE_B:
			source_pos[0] = 0.7;//cvs->kalman_pos->x;
			source_pos[1] = 1.0;//cvs->kalman_pos->y;
			goal_pos[0] = -0.4;
			goal_pos[1] = -0.6;
			path = path_planning_compute(cvs, source_pos, goal_pos);
			strat->main_state = GAME_STATE_C;
			break;

		case GAME_STATE_C:
			follow_path(cvs, path);
			break;

		case GAME_STATE_D:
			source_pos[0] = -0.8;//cvs->kalman_pos->x;
			source_pos[1] = +0.0;//cvs->kalman_pos->y;
			goal_pos[0] = -0.4;
			goal_pos[1] = -0.6;
			path = path_planning_compute(cvs, source_pos, goal_pos);
			strat->main_state = GAME_STATE_C;
			break;

		case GAME_STATE_E:
			cvs->main_state = STOP_END_STATE;
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_WAIT:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
