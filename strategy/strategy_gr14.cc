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

vector<array<float,2> > path_;
path_t tt;

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

	// target_path_t target_path1[4]  = {{+0.7,+0.9},{+0.7,+0.8},{+0.7,+0.7},{0.7,0.6}};
	// target_path_t target_path2[10] = {{+0.65,+0.6},{+0.55,+0.6},{+0.45,+0.6},{0.25,0.6},{+0.25,+0.7},{+0.25,+0.8},{+0.25,+0.9},{+0.25,+1.0},{+0.25,+1.1},{+0.25,+1.25}};
	// target_path_t target_path3[6]  = {{+0.1,+1.1},{+0.0,+1.0},{-0.1,+0.9},{-0.2,+0.8},{-0.3,+0.7},{-0.4,+0.6}};
	// target_path_t target_path4[6]  = {{-0.5,+0.5},{-0.6,+0.4},{-0.7,+0.3},{-0.8,+0.2},{-0.8,+0.1},{-0.8,+0.0}};
	// target_path_t target_path5[6]  = {{-0.8,-0.1},{-0.8,-0.2},{-0.7,-0.3},{-0.6,-0.4},{-0.5,-0.5},{-0.4,-0.6}};
	// target_path_t target_path6[6]  = {{-0.3,-0.7},{-0.2,-0.8},{-0.1,-0.9},{+0.0,-1.0},{+0.1,-1.1},{+0.25,-1.25}};

	// target_t target[6] = {{4,target_path1},{10,target_path2},{6,target_path3},{6,target_path4},{6,target_path5},{6,target_path6}};

	// path.nb_target = 6;
	// path.targets = target;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;

	array<float, 2> source_pos;
	array<float, 2> goal_pos;

	switch (strat->main_state)
	{
		case GAME_STATE_A:
			// follow_path(cvs, &path);
			break;

		case GAME_STATE_B:
			printf("initialization of path...");
			source_pos[0] = 0.7;//cvs->kalman_pos->x;
			source_pos[1] = 1.0;//cvs->kalman_pos->y;
			goal_pos[0] = -0.4;
			goal_pos[1] = -0.6;
			path_ = path_planning_compute(cvs, source_pos, goal_pos);
			strat->main_state = GAME_STATE_C;
			printf("Done\n");

			printf("Size in follow_path: %lu\n", path_.size());
			for (size_t l = 0; l< path_.size(); l++)
			{
				printf("Going for x:%.3f y: %.3f\n", path_[l][0], path_[l][1]);
			}
			// printf("Size in startegy: %lu\n", path_.size());
			// for (size_t l = 0; l< path_.size(); l++)
			// {
			// 	printf("Going for x:%.3f y: %.3f\n", path_[l][0], path_[l][1]);
			// }
			break;

		case GAME_STATE_C:
			// printf("follow_path\n");
			follow_path(cvs, path_);
			tt.last_t = inputs->t;
			// printf("Following path\n");
			break;

		case GAME_STATE_D:
			printf("initialization of path...");
			source_pos[0] = -0.8;//cvs->kalman_pos->x;
			source_pos[1] = +0.0;//cvs->kalman_pos->y;
			goal_pos[0] = -0.4;
			goal_pos[1] = -0.6;
			path_ = path_planning_compute(cvs, source_pos, goal_pos);
			strat->main_state = GAME_STATE_C;
			printf("Done\n");

			printf("Size in follow_path: %lu\n", path_.size());
			for (size_t l = 0; l< path_.size(); l++)
			{
				printf("Going for x:%.3f y: %.3f\n", path_[l][0], path_[l][1]);
			}
			break;

		case GAME_STATE_E:
			printf("Stop Machine! Atteint la fin\n");
			cvs->main_state = STOP_END_STATE;
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_WAIT:
			speed_regulation(cvs, 0.0, 0.0);
			if ( inputs->t - tt.last_t > 1.5)
			{
				tt.last_t = inputs->t;
				strat->main_state = GAME_STATE_D;
			}
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
