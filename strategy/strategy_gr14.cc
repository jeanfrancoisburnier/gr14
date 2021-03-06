#include "strategy_gr14.h"
#include "path_planning_gr14.h"
#include "speed_regulation_gr14.h"
#include "path_regulation_gr14.h"
#include "path_planning_gr14.h"
#include "init_pos_gr14.h"
#include "opp_pos_gr14.h"
#include "odometry_gr14.h"
#include "node_gr14.h"
#include "useful_gr14.h"
#include <vector>
#include <array>
#include <math.h>

using namespace std;

NAMESPACE_INIT(ctrlGr14);
#define FORWARD true;
#define BACKWARD false;
#define TAU 0.01
#define MARGIN_POS 0.050 //if its last_position minus its actual is inferior to this margin during too much time its considered blocked
#define RECOMPUTE_PATH_T 0.1

static double last_call = 0;//used in deblock_robot to construct or delta_t for the wheel command

vector<array<float,2> > path;

/*! \brief intitialize the strategy structure
 *
 * \return strategy structure initialized
 */
Strategy* init_strategy(CtrlStruct *cvs)
{
	Strategy *strat;

	strat = (Strategy*) malloc(sizeof(Strategy));

	strat->current_target_id = 0;
	strat->current_point_id = 0;

	strat->last_t_wait = 0;
	strat->last_t_path = 0;

	strat->carrying_target_id[0] = -1;
	strat->carrying_target_id[1] = -1;

	strat->prev_nb_target_carrying = 0;

	strat->attempts = 0;

	strat->remaining_targets = 8;

	strat->init_flag = true;

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
	static int counter_times_fix = 0;
	static bool orientation = FORWARD;
	//Static variables used in GAME_STATE_BLOCKED
	static bool first_call_block = true;
	static double time_first_call_block = 0.0;
	static float last_pos_robot[2];
	static int nb_still_fix = 0;

	// variables declaration
	Strategy *strat;
	CtrlIn *inputs;
	uint8_t target_id;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;
	target_id = strat->current_target_id;

	update_target_status(cvs);

	array<float, 2> source_pos;
	array<float, 2> goal_pos;

	if (strat->init_flag && cvs->team_id == TEAM_A)
	{
		strat->target[0].x = -0.8;
		strat->target[0].y = +0.0;
		strat->target[0].status = TARGET_FREE;

		strat->target[1].x = +0.25;
		strat->target[1].y = +1.25;
		strat->target[1].status = TARGET_FREE;

		strat->target[2].x = +0.1;
		strat->target[2].y = +0.0;
		strat->target[2].status = TARGET_FREE;

		strat->target[3].x = +0.25;
		strat->target[3].y = -1.25;
		strat->target[3].status = TARGET_FREE;

		strat->target[4].x = -0.4;
		strat->target[4].y = +0.6;
		strat->target[4].status = TARGET_FREE;

		strat->target[5].x = +0.7;
		strat->target[5].y = +0.6;
		strat->target[5].status = TARGET_FREE;

		strat->target[6].x = -0.4;
		strat->target[6].y = -0.6;
		strat->target[6].status = TARGET_FREE;

		strat->target[7].x = +0.7;
		strat->target[7].y = -0.6;
		strat->target[7].status = TARGET_FREE;

		strat->start_base.x = +0.7;
		strat->start_base.y = +0.6;

		strat->target_base.x = -0.8;
		strat->target_base.y = -1.2;

		strat->init_flag = false;
	}
	else if (strat->init_flag && cvs->team_id == TEAM_B)
	{
		strat->target[0].x = -0.8;
		strat->target[0].y = +0.0;
		strat->target[0].status = TARGET_FREE;

		strat->target[1].x = +0.25;
		strat->target[1].y = -1.25;
		strat->target[1].status = TARGET_FREE;

		strat->target[2].x = +0.1;
		strat->target[2].y = +0.0;
		strat->target[2].status = TARGET_FREE;

		strat->target[3].x = +0.25;
		strat->target[3].y = +1.25;
		strat->target[3].status = TARGET_FREE;

		strat->target[4].x = -0.4;
		strat->target[4].y = -0.6;
		strat->target[4].status = TARGET_FREE;

		strat->target[5].x = +0.7;
		strat->target[5].y = -0.6;
		strat->target[5].status = TARGET_FREE;

		strat->target[6].x = -0.4;
		strat->target[6].y = +0.6;
		strat->target[6].status = TARGET_FREE;

		strat->target[7].x = +0.7;
		strat->target[7].y = +0.6;
		strat->target[7].status = TARGET_FREE;

		strat->start_base.x = +0.7;
		strat->start_base.y = -0.6;

		strat->target_base.x = -0.8;
		strat->target_base.y = +1.2;

		strat->init_flag = false;
	}
	switch (strat->main_state)
	{
		case GAME_STATE_COMPUTE_PATH:
			cvs->outputs->flag_release = 0;
			source_pos[0] = cvs->kalman_pos->x;
			source_pos[1] = cvs->kalman_pos->y;

			if (inputs->nb_targets < 2 && strat->remaining_targets >= 1)
			{
				strat->status = STRAT_TARGET;
				int k = 0;
				while (strat->target[target_id].status == TARGET_WON || strat->target[target_id].status == TARGET_STOLEN)
				{
					update_current_target_id(strat);
					target_id = strat->current_target_id;
				}
				goal_pos[0] = strat->target[target_id % 8].x;
				goal_pos[1] = strat->target[target_id % 8].y;

			}
			else
			{
				strat->status = STRAT_SCORING;
				goal_pos[0] = strat->target_base.x;
				goal_pos[1] = strat->target_base.y;
			}

			if (strat->remaining_targets < 0)
			{
				// printf("END GAME at t = %.2f!\n", cvs->inputs->t);
				strat->main_state = GAME_STATE_E;
				return;
			}
			path = path_planning_compute(cvs, source_pos, goal_pos);
			if (!path.empty())//if we're not able to compute a path, we do it again until we got one
			{
				strat->main_state = GAME_STATE_GO_TO_GOAL;
			}
			strat->last_t_path = inputs->t;
			break;

		case GAME_STATE_GO_TO_GOAL:
			if (inputs->t - strat->last_t_path >= RECOMPUTE_PATH_T)
			{
				//if at the same position as the last call
				if(fabs(cvs->kalman_pos->x - last_pos_robot[X]) < MARGIN_POS
					&& fabs(cvs->kalman_pos->y - last_pos_robot[Y]) < MARGIN_POS)//if at the same position as the last call
				{
					counter_times_fix++;
					if(counter_times_fix >= 3.0/RECOMPUTE_PATH_T)//means it's been 3 seconds it was blocked
					{
						counter_times_fix = 0;
						strat->main_state = GAME_STATE_BLOCKED;
						break;
					}
				}
				else//will be necessarily called at least one time --> no need to initialize last_pos_robot
					//update last_pos_robot if it's not blocked
				{
					counter_times_fix = 0;
					last_pos_robot[X] = cvs->kalman_pos->x;
					last_pos_robot[Y] = cvs->kalman_pos->y;
				}

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
					update_current_target_id(strat);
					strat->current_point_id = 0;
					strat->attempts = 0;

					if (strat->remaining_targets <= 1)
					{
						strat->remaining_targets--;
					}
 				}
 				else if (strat->attempts > 1)
 				{
 					strat->target[strat->current_target_id].status = TARGET_STOLEN;
 					strat->remaining_targets--;
 					strat->attempts = 0;
 				}
 				else
 				{
 					strat->attempts++;
 				}
 				strat->main_state = GAME_STATE_COMPUTE_PATH;
 			}
			break;

		case GAME_STATE_BLOCKED: //if the robot stay more than 2 second it means it's blocked
			if(first_call_block == true)
			{
				first_call_block = false;
				time_first_call_block = inputs->t;
			}
			else
			{
				deblock_robot(cvs, orientation);
				if((inputs->t - time_first_call_block) > 2.0)//if it's been already 1 second we try to deblock ourself --> we might be free
				{
					if(fabs(cvs->kalman_pos->x - last_pos_robot[X]) < MARGIN_POS
						&& fabs(cvs->kalman_pos->y - last_pos_robot[Y]) < MARGIN_POS)//TJRS bloqué
					{
						orientation = BACKWARD;
						deblock_robot(cvs, orientation);
					}
					else
					{
						orientation = FORWARD;
						strat->main_state = GAME_STATE_COMPUTE_PATH;//return do what it was supposed to do when deblocked
					}
					first_call_block = true;
					time_first_call_block = 0;
				}
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


//to call each 1.5 sec to see if the opponent just pass above the target or stopped to take it
void update_target_status(CtrlStruct *cvs)
{
	Strategy *strat;
	strat  = cvs->strat;

	static int target_occupied[2] = {-1};
	static double last_t_update[2];

	int n = cvs->inputs->nb_opponents;
	vector<Obstacles> moving_obstacles = update_moving_obstacles(cvs);//update the posîtion of the opponent robot
	for(int j=0; j<n; j++)
	{
		int i;
		for(i = 0; i<8; i++)
		{
			if( (strat->target[i].status != TARGET_STOLEN) && (strat->target[i].status != TARGET_WON) )
			{
				if( (strat->target[i].x > moving_obstacles[j].first_corner[X]) &&
						(strat->target[i].x < moving_obstacles[j].second_corner[X]) &&
						(strat->target[i].y < moving_obstacles[j].first_corner[Y]) &&
						(strat->target[i].y > moving_obstacles[j].second_corner[Y]) )
				{
					if (target_occupied[j] != i)
					{
						target_occupied[j] = i;
						last_t_update[j] = cvs->inputs->t;
					}
					if (cvs->inputs->t - last_t_update[j] > 1.3)
					{
						strat->target[i].status = TARGET_STOLEN;
						strat->remaining_targets--;
						target_occupied[j] = -1;
						last_t_update[j] = cvs->inputs->t;

					}
					break;
				}
			}
		}
		if (i == 8)
		{
			target_occupied[j] = -1;
		}
	}
}



//will indicate the robot to go at the opposite as the direction it was originally following
void deblock_robot(CtrlStruct *cvs, bool orient)
{
	if(orient)//we go in the opposit direction
	{
		speed_regulation(cvs, -10, -10);
	}
	else//BACKWARDS, we go in the opposit direction
	{
		speed_regulation(cvs, 10, 10);
	}
}





/*return false if the opponent isn't too close or if we go away from it and true in the other case
level correspond to the level of magnitude of the distance to the opponent
*/
bool test_opponent_too_close(CtrlStruct *cvs, double level)
{
	double security_dist;

	static int n = cvs->inputs->nb_opponents;

	CtrlIn *inputs;
	KalmanStruct *kalman_pos;
	OpponentsPosition *opp_pos;

	inputs  = cvs->inputs;
	kalman_pos = cvs->kalman_pos;
	opp_pos = cvs->opp_pos;


	security_dist = level;
	double new_distance = 0.;


	switch(n)
	{
		case 0 : //if no opponent
			return false;



		case 1 : //If only one opponent
			static double last_distance = pow(opp_pos->x[0]-kalman_pos->x, 2) + pow(opp_pos->y[0]-kalman_pos->y, 2);

			new_distance = pow(opp_pos->x[0] - kalman_pos->x, 2) + pow(opp_pos->y[0] - kalman_pos->y, 2);

			if(new_distance < security_dist)//if we're too close to the enemy
			{
				// printf("Too close, security = %1f, last_distance = %3f\n", security_dist, last_distance);
				if( (last_distance - new_distance)  > -MARGIN_POS)//If we go near the opponent

				{
					last_distance = new_distance;
					return true;
				}
				else//we go away from the opponent
				{
					last_distance = new_distance;
					return false;
				}
			}
			else
			{
				last_distance = new_distance;
				return false;
			}



		case 2 ://if two opponents
			static double last_distance_1 = pow(opp_pos->x[0]-kalman_pos->x, 2) + pow(opp_pos->y[0]-kalman_pos->y, 2);
			static double last_distance_2 = pow(opp_pos->x[1]-kalman_pos->x, 2) + pow(opp_pos->y[1]-kalman_pos->y, 2);

			new_distance = pow(opp_pos->x[0] - kalman_pos->x, 2) + pow(opp_pos->y[0] - kalman_pos->y, 2);//distance from the first opponent
			if(new_distance < security_dist)//if we're too close to the enemy n°1
			{
				if( (last_distance_1 - new_distance)  > -MARGIN_POS)//If we go near the opponent n°1
				{
					last_distance_1 = new_distance;
					return true;
				}
			}

			new_distance = pow(opp_pos->x[1] - kalman_pos->x, 2) + pow(opp_pos->y[1] - kalman_pos->y, 2);//distance from the second opponent
			if(new_distance < security_dist)//if we're too close to the enemy n°2
			{
				if( (last_distance_2 - new_distance)  > -MARGIN_POS)//If we draw near the opponent n°2
				{
					last_distance_2 = new_distance;
					return true;
				}
				else//we go away from the opponent n°2
				{
					last_distance_2 = new_distance;
					return false;
				}
			}
			else
			{
				return false;
			}
	}
}


NAMESPACE_CLOSE();
