#include "path_regulation_gr14.h"
#include "useful_gr14.h"
#include "speed_regulation_gr14.h"
#include "init_pos_gr14.h"
#include "strategy_gr14.h"
#include "set_output.h"

NAMESPACE_INIT(ctrlGr14);

#define TAU 0.03
#define RADIUS_TOL 0.0144//0.0025

#define ALPHA 	M_PI/32
#define BETA 	M_PI/16
#define GAMMA 	M_PI/8
#define DELTA 	M_PI/4

static double last_t = 0;

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
// void follow_path(CtrlStruct *cvs, path_t* tab)
void follow_path(CtrlStruct *cvs, vector<array<float,2> > path)
{
	static size_t i=0;
	// printf("Getting target %d\n", i+1);
	// Intermediate position to reach
	// if (path.size() < 2)
	// {
	// 	return;
	// }
	float x_target = path[i][0];
	float y_target = path[i][1];

	// printf("Going for x:%.3f y: %.3f\n", x_target, y_target);
	// printf("Size in follow_path: %lu\n", path.size());
	// // for (size_t l = 0; l< path.size(); l++)
	// // {
	// // 	printf("Going for x:%.3f y: %.3f\n", path[l][0], path[l][1]);
	// // }
	
	float distance = 0.0;

	KalmanStruct *kalman_pos;
	kalman_pos = cvs->kalman_pos;

	double delta_t = cvs->inputs->t - last_t;
	last_t = cvs->inputs->t;

	// Vector from robot to target
	float vector_x = x_target-kalman_pos->x;
	float vector_y = y_target-kalman_pos->y;

	distance = pow(vector_x,2)+pow(vector_y,2);

	if (distance < RADIUS_TOL) 
	{
		printf("!!!!!!!!!!!!!!!!!!!Via Reached!!!!!!!!!!!!!!!!!!!\n");
		set_output(path[i][0],"x_path");
		set_output(path[i][1],"y_path");
		set_output(path.size(),"path_size");
		i++;
	}
	// if (i >= tab->targets[tab->current_target].nb_via)
	if (i >= path.size())
	{
		printf("!!!!!!!!!!!!!!!!!!!Target Reached!!!!!!!!!!!!!!!!!!!\n");
		i = 0;
		static bool first = false;
		if (first)
		{
			cvs->strat->main_state = GAME_STATE_WAIT;//GAME_STATE_D;
			first = false;
		}
		else
		{
			cvs->strat->main_state = GAME_STATE_E;
		}

		return;
	}
	// printf("i is %d\n", i);

	float beta = atan2(vector_y,vector_x); // absolute angle of target
	// printf("Angle beta: %f\n", beta);

	float gamma = limit_angle(beta-kalman_pos->theta); // relative angle of target with robot

	float l_speed = 0;
	float r_speed = 0;

	reach_single_target(gamma, &l_speed, &r_speed);

	r_speed = first_order_filter(cvs->inputs->r_wheel_speed, r_speed, TAU, delta_t);
	l_speed = first_order_filter(cvs->inputs->l_wheel_speed, l_speed, TAU, delta_t);

	set_plot(r_speed,"R_SPEED");
	set_plot(l_speed,"L_SPEED");

	speed_regulation(cvs, r_speed, l_speed);
}

void reach_single_target(float gamma, float *l_speed, float *r_speed)
{
	// printf("Angle gamma: %f\n", gamma);
	if (gamma > -ALPHA/2 && gamma < ALPHA/2) // target in front of robot
	{
		*l_speed = +10.0;
		*r_speed = +10.0;
		// printf("Dré!\n");
	}
	else if (gamma > ALPHA/2 && gamma < BETA+ALPHA/2) // target in front of robot
	{
		*l_speed = +5.0;
		*r_speed = +10.0;
		// printf("Tourne un peu à gauche\n");
	}
	else if (gamma > BETA+ALPHA/2 && gamma < GAMMA+BETA+ALPHA/2) // target in front of robot
	{
		*l_speed = +2.5;
		*r_speed = +10.0;
		// printf("Tourne un peu plus à gauche\n");
	}
	else if (gamma > GAMMA+BETA+ALPHA/2 && gamma < DELTA+GAMMA+BETA+ALPHA/2) // target in front of robot
	{
		*l_speed = -8.0;
		*r_speed = +8.0;
		// printf("Tourne bien à gauche\n");
	}
	else if (gamma > DELTA+GAMMA+BETA+ALPHA/2 && gamma < M_PI) // target in front of robot
	{
		*l_speed = -10.0;
		*r_speed = +10.0;
		// printf("Tourne bcp à gauche\n");
	}
	else if (gamma > -M_PI && gamma < -(DELTA+GAMMA+BETA+ALPHA/2))
	{
		*l_speed = +10.0;
		*r_speed = -10.0;
		// printf("Tourne bcp à droite\n");
	}
	else if (gamma > -(DELTA+GAMMA+BETA+ALPHA/2) && gamma < -(GAMMA+BETA+ALPHA/2)) // target in front of robot
	{
		*l_speed = +8.0;
		*r_speed = -8.0;
		// printf("Tourne bien à droite\n");
	}
	else if (gamma > -(GAMMA+BETA+ALPHA/2) && gamma < -(BETA+ALPHA/2)) // target in front of robot
	{
		*l_speed = +10.0;
		*r_speed = +2.5;
		// printf("Tourne un peu plus à droite\n");
	}
	else if (gamma > -(BETA+ALPHA/2) && gamma < -ALPHA/2) // target in front of robot
	{
		*l_speed = +10.0;
		*r_speed = +5.0;
		// printf("Tourne un peu à droite\n");
	}
	else
	{
		printf("reach single target failure\n");
	}
}

NAMESPACE_CLOSE();
