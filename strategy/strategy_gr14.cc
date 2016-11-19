#include "strategy_gr14.h"
#include "path_planning_gr14.h"
#include "speed_regulation_gr14.h"
#include "path_regulation_gr14.h"
#include "init_pos_gr14.h"
#include "opp_pos_gr14.h"
#include "odometry_gr14.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr14);

/*! \brief intitialize the strategy structure
 * 
 * \return strategy structure initialized
 */
Strategy* init_strategy()
{
	Strategy *strat;

	strat = (Strategy*) malloc(sizeof(Strategy));

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

	target_coordinate_t target_coordinate[4];
	target_coordinate[0].x = +0.7; target_coordinate[0].y = +0.6;
	target_coordinate[1].x = +0.25; target_coordinate[1].y = +0.6;
	target_coordinate[2].x = +0.25; target_coordinate[2].y = +1.25;
	target_coordinate[3].x = -0.4; target_coordinate[3].y = +0.6;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;

	switch (strat->main_state)
	{
		case GAME_STATE_A:
			follow_path(cvs, target_coordinate, 4);
			// speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_B:
			speed_regulation(cvs, 0.0, 0.0);
			printf("Stop Machine! Atteint la fin\n");
			break;

		case GAME_STATE_C:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_D:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_E:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
