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

#define TAU 0.01
#define MARGIN_POS 0.050 //if its last_position minus its actual is inferior to this margin during too much time its considered blocked
static double last_call = 0;//used in deblock_robot to construct or delta_t for the wheel command


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
					strat->current_target_id = (strat->current_target_id + 1) % 8;
					target_id = strat->current_target_id;
				}
				goal_pos[0] = strat->target[target_id % 8].x;
				goal_pos[1] = strat->target[target_id % 8].y;
				printf("Going from x: %.3f y: %.3f\n", source_pos[0], source_pos[1]);
				printf("Going for target %d at x: %.3f y: %.3f\n", target_id+1, goal_pos[0], goal_pos[1]);

			}
			else
			{
				strat->status = STRAT_SCORING;
				goal_pos[0] = strat->target_base.x;
				goal_pos[1] = strat->target_base.y;
			}

			// while(test_if_goal_is_set_on_opponent(cvs, goal_pos))//if goal on an opponent, go to the next target
			// {
			// 	target_id++;
			// 	if(target_id == 8)
			// 	{
			// 		target_id = 0;
			// 	}
			// 	goal_pos[0] = strat->target[target_id].x;
			// 	goal_pos[1] = strat->target[target_id].y;
			// }
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

				//if at the same position as the last call
				if(abs(cvs->kalman_pos->x - last_pos_robot[X]) < MARGIN_POS
					&& abs(cvs->kalman_pos->y - last_pos_robot[Y]) < MARGIN_POS)//if at the same position as the last call
				{
					counter_times_fix++;
					if(counter_times_fix >= 10)//means it's been 3 seconds it was blocked
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
					
				// printf("=========>We need to compute a path!!!\n");
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

		case GAME_STATE_BLOCKED: //if the robot stay more than 2 second it means it's blocked
			printf("In GAME_STATE_BLOCKED\n");
			if(first_call_block == true)
			{
				first_call_block = false;
				time_first_call_block = inputs->t;
			}
			else
			{
				deblock_robot(cvs, path, orientation);
				if((inputs->t - time_first_call_block) > 2.0)//if it's been already 2 second we try to deblock ourself --> we might be free
				{
					//path = path_planning_compute(cvs, source_pos, goal_pos);
					if(abs(cvs->kalman_pos->x - last_pos_robot[X]) < MARGIN_POS
						&& abs(cvs->kalman_pos->y - last_pos_robot[Y]) < MARGIN_POS)//TJRS bloqué
					{
						orientation = BACKWARD;
						deblock_robot(cvs, path, orientation);
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


//to call each 1.5 sec to see if the opponent just pass above the target or stopped to take it
void update_target_status(CtrlStruct *cvs)
{
	Strategy *strat;
	strat  = cvs->strat;
	
	static bool was_on_it[8] = {false};

	int n = cvs->inputs->nb_opponents;
	vector<Obstacles> moving_obstacles = update_moving_obstacles(cvs);//update the posîtion of the opponent robot

	for(int j=0; j<n; j++)
	{
		for(int i=0; i<NB_TARGET; i++)
		{
			if( (strat->target[i].status != TARGET_STOLEN) || (strat->target[i].status != TARGET_WON) )
			{
				if( (strat->target[i].x > moving_obstacles[j].first_corner[X]) && 
						(strat->target[i].x < moving_obstacles[j].second_corner[X]) && 
						(strat->target[i].y < moving_obstacles[j].first_corner[Y]) && 
						(strat->target[i].y > moving_obstacles[j].second_corner[Y]) )
				{
					if(was_on_it[i] == false)
					{
						was_on_it[i] = true;
					}
					else
					{
						strat->target[i].status = TARGET_STOLEN;
						was_on_it[i] = false;
					}
				}	
			}
		}	
	}
}


//will indicate the robot to go at the opposite as the direction it was originally following
void deblock_robot(CtrlStruct *cvs, vector<array<float,2> > path, bool orient)
{
	printf("In deblock_robot\n");
	/*ouble delta_t = cvs->inputs->t - last_call;
	last_call = cvs->inputs->t;


	int i = (int) path.size() - 1;

	// Position of the next point to reach
	double x_point = path[i][0];
	double y_point = path[i][1];


	// Vector from robot to target
	double vector_x = x_point-cvs->kalman_pos->x;
	double vector_y = y_point-cvs->kalman_pos->y;

	double beta  = atan2(vector_y,vector_x); // absolute angle of target
	double gamma = limit_angle(beta-cvs->kalman_pos->theta); // relative angle of target with robot

	double l_speed = 0;
	double r_speed = 0;

	get_new_speed(gamma, &l_speed, &r_speed);
	// Filter new speed command
	r_speed = first_order_filter(cvs->inputs->r_wheel_speed, r_speed, TAU, delta_t);
	l_speed = first_order_filter(cvs->inputs->l_wheel_speed, l_speed, TAU, delta_t);*/

	// Set new speed in the opposite wanted direction 
	//speed_regulation(cvs, -1000*r_speed, -1000*l_speed);

	if(orient)//we go in the opposit direction
	{
		speed_regulation(cvs, -10, -10);
	}
	else//BACKWARDS, we go in the opposit direction
	{
		speed_regulation(cvs, 10, 10);
	}
}

NAMESPACE_CLOSE();
