#include "path_regulation_gr14.h"
#include "useful_gr14.h"
#include "speed_regulation_gr14.h"
#include "init_pos_gr14.h"
#include "strategy_gr14.h"

NAMESPACE_INIT(ctrlGr14);

#define TAU 0.4
#define RADIUS_TOL 0.0144

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs, target_coordinate_t* tab, uint8_t size)
// void follow_path(CtrlStruct *cvs)
{
	static int i=0;
	// Intermediate position to reach
	float x_target = tab[i].x;
	float y_target = tab[i].y;
	
	// float x_target = -0.400;
	// float y_target = +0.600;
	
	float distance = 0.0;

	KalmanStruct *kalman_pos;
	kalman_pos = cvs->kalman_pos;

	double delta_t = cvs->inputs->t - kalman_pos->last_t;

	// Vector from robot to target
	float vector_x = x_target-kalman_pos->x;
	float vector_y = y_target-kalman_pos->y;

	distance = pow(vector_x,2)+pow(vector_y,2);

	if (distance < RADIUS_TOL) 
	{
		i++;
	}
	if (i > size)
	{
		cvs->strat->main_state = GAME_STATE_B;
		i = 0;
		return;
	}
	// printf("i is %d\n", i);

	float beta = atan2(vector_y,vector_x); // absolute angle of target
	// printf("Angle beta: %f\n", beta);

	float gamma = limit_angle(beta-kalman_pos->theta); // relative angle of target with robot

	float l_speed = 0;
	float r_speed = 0;

	reach_single_target(gamma, &l_speed, &r_speed);

	// r_speed = first_order_filter(cvs->inputs->r_wheel_speed, r_speed, TAU, delta_t);
	// l_speed = first_order_filter(cvs->inputs->l_wheel_speed, l_speed, TAU, delta_t);

	speed_regulation(cvs, r_speed, l_speed);
}

void reach_single_target(float gamma, float *l_speed, float *r_speed)
{
	// printf("Angle gamma: %f\n", gamma);
	if (gamma > -M_PI/8.0 && gamma < M_PI/8.0) // target in front of robot
	{
		*l_speed = +5.0;
		*r_speed = +5.0;
		printf("Dré!\n");
	}
	else if (gamma > M_PI/8.0 && gamma < 3*M_PI/8.0) // target in front of robot
	{
		*l_speed = -1.0;
		*r_speed = +5.0;
		printf("Tourne un peu à gauche\n");
	}
	else if (gamma > 3*M_PI/8.0 && gamma < 5*M_PI/8.0) // target in front of robot
	{
		*l_speed = -5.0;
		*r_speed = +5.0;
		printf("Tourne un peu plus à gauche\n");
	}
	else if (gamma > 5*M_PI/8.0 && gamma < 7*M_PI/8.0) // target in front of robot
	{
		*l_speed = -8.0;
		*r_speed = +8.0;
		printf("Tourne bien à gauche\n");
	}
	else if (gamma > 7*M_PI/8.0 && gamma < M_PI) // target in front of robot
	{
		*l_speed = -10.0;
		*r_speed = +10.0;
		printf("Tourne bcp à gauche\n");
	}
	else if (gamma > -M_PI && gamma < -7*M_PI/8.0)
	{
		*l_speed = +10.0;
		*r_speed = -10.0;
		printf("Tourne bcp à droite\n");
	}
	else if (gamma > -7*M_PI/8.0 && gamma < -5*M_PI/8.0) // target in front of robot
	{
		*l_speed = +8.0;
		*r_speed = -8.0;
		printf("Tourne bien à droite\n");
	}
	else if (gamma > -5*M_PI/8.0 && gamma < -3*M_PI/8.0) // target in front of robot
	{
		*l_speed = +5.0;
		*r_speed = -5.0;
		printf("Tourne un peu plus à droite\n");
	}
	else if (gamma > -3*M_PI/8.0 && gamma < -M_PI/8.0) // target in front of robot
	{
		*l_speed = +5.0;
		*r_speed = -1.0;
		printf("Tourne un peu à droite\n");
	}
	else
	{
		printf("reach single target failure\n");
	}
}

NAMESPACE_CLOSE();
