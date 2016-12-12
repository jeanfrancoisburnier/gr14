#include "path_regulation_gr14.h"
#include "useful_gr14.h"
#include "speed_regulation_gr14.h"
#include "init_pos_gr14.h"
#include "strategy_gr14.h"
#include "set_output.h"
#include "ColorSensor.hh"

#include <math.h>

NAMESPACE_INIT(ctrlGr14);

#define TAU 0.01
#define RADIUS_TOL 0.0144
#define RADIUS_TOL_TAR 0.003
#define RADIUS_TOL_NEW 0.05

#define ALPHA 	M_PI/32
#define BETA 	M_PI/16
#define GAMMA 	M_PI/4

static int K = 3;

static double last_t = 0;

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs, vector<array<float,2> > path)
{
	// Robot current position
	KalmanStruct* kalman_pos = cvs->kalman_pos;
	Strategy* strat = cvs->strat;

	size_t i = strat->current_point_id;

	// Position of the next point to reach
	float x_point = path[i][0];
	float y_point = path[i][1];

	// printf("Going for the point x: %.3f \t y: %.3f\n", x_point, y_point);

	double delta_t = cvs->inputs->t - last_t;
	last_t = cvs->inputs->t;

	// Vector from robot to target
	float vector_x = x_point-kalman_pos->x;
	float vector_y = y_point-kalman_pos->y;
		
	if (i < path.size() - 1)
	{
		K = 3;

		if (pow(vector_x,2)+pow(vector_y,2) < RADIUS_TOL) 
		{
			set_output(path[i][0],"x_path");
			set_output(path[i][1],"y_path");
			set_output(path.size(),"path_size");
			strat->current_point_id++;
		}
	}
	else if (i == path.size() - 1 && pow(vector_x,2)+pow(vector_y,2) < RADIUS_TOL_NEW)
	{
		K = 1;

		if (pow(vector_x,2)+pow(vector_y,2) < RADIUS_TOL_TAR)
		{
			printf("i is:%lu and size is: %lu\n",i, path.size());
			strat->current_point_id = 0;
			if (cvs->strat->current_target_id <= 7)
			{

				if (cvs->inputs->color_seen == MAP_BLUE)
				{
					cvs->outputs->flag_release = 1;
					printf("Target Released!\n");
				}
				else
				{
					strat->current_target_id = (strat->current_target_id + 1)%8;
					printf("Target Reached! Robots is at x: %.3f \t y: %.3f\n", kalman_pos->x, kalman_pos->y);
					printf("Deltax: %.3f \t Deltay: %.3f\n", vector_x, vector_y);
					printf("XPoint: %.3f \t YPoint: %.3f\n", x_point, y_point);
					printf("Distance2: %.3f\n", pow(vector_x,2)+pow(vector_y,2));
				}
				strat->main_state = GAME_STATE_WAIT;
			}
			return;
		}
	}

	float beta  = atan2(vector_y,vector_x); // absolute angle of target
	float gamma = limit_angle(beta-kalman_pos->theta); // relative angle of target with robot

	float l_speed = 0;
	float r_speed = 0;

	get_new_speed(gamma, &l_speed, &r_speed);

	// Filter new speed command
	r_speed = first_order_filter(cvs->inputs->r_wheel_speed, r_speed, TAU, delta_t);
	l_speed = first_order_filter(cvs->inputs->l_wheel_speed, l_speed, TAU, delta_t);
	// printf("XPoint: %.3f \t YPoint: %.3f\n", x_point, y_point);
	// Set new speed
	speed_regulation(cvs, r_speed, l_speed);
}

void get_new_speed(float gamma, float *l_speed, float *r_speed)
{
	int quadrant = 0;
	int b = 0;

	double speed[4] = {10.0,5.0,2.0,-10.0};

	if (gamma > 0 && gamma <= M_PI/2)
	{
		quadrant = 0;	
	}
	else if (gamma > M_PI/2 && gamma <= M_PI)
	{
		quadrant = 1;
	}
	else if (gamma >= -M_PI && gamma <= -M_PI/2)
	{
		quadrant = 3;
	}
	else if (gamma > -M_PI/2 && gamma <= 0)
	{
		quadrant = 2;
	}
	else
	{
		printf("Error\n");
		return;
	}

	gamma = fabs(gamma);
    if (gamma > M_PI/2)
    {
        gamma = M_PI - gamma;
    }

    // printf("gamma is:%.3f\n", gamma);
	if (gamma < ALPHA/2)
	{
		b = 0;
	}
	else if (gamma < ALPHA/2+BETA)
	{
		b = 1;
	}
	else if (gamma < ALPHA/2+BETA+GAMMA)
	{
		b = 2;
	}
	else
	{
		b = 3;
	}

	if (quadrant >> 1)
	{
		// printf("In quandrant 2 or 3\n");
		*l_speed = 10.0;
		*r_speed = speed[b];
	}
	else
	{
		*l_speed = speed[b];
		*r_speed = 10.0;
	}

	if (quadrant & 1)
	{
		// printf("In quandrant 1 or 3\n");
		*l_speed *= -1.0;
		*r_speed *= -1.0;
	}

	*l_speed *= K;
	*r_speed *= K;

	// printf("Quadrant: %d, b = %d \t l: %.3f, r: %.3f\n", quadrant, b, *l_speed, *r_speed);

	// // Zone 1
	// if (gamma > -ALPHA/2 && gamma <= ALPHA/2) // point in front of robot
	// {
	// 	*l_speed = +10.0*K;
	// 	*r_speed = +10.0*K;
	// }
	// // Zone 2
	// else if (gamma > ALPHA/2 && gamma <= BETA+ALPHA/2) // point a bit on the left
	// {
	// 	*l_speed = +5.0*K;
	// 	*r_speed = +10.0*K;
	// }
	// // Zone 3
	// else if (gamma > BETA+ALPHA/2 && gamma <= GAMMA+BETA+ALPHA/2) // point a bit more on the left
	// {
	// 	*l_speed = +2.0*K;
	// 	*r_speed = +10.0*K;
	// }
	// else if (gamma > GAMMA+BETA+ALPHA/2 && gamma <= M_PI/2)
	// {
	// 	*l_speed = -5.0*K;
	// 	*r_speed = +5.0*K;
	// }
	// // Zone 4
	// else if (gamma > M_PI/2 && gamma <= M_PI-(GAMMA+BETA+ALPHA/2)) // point on the left
	// {
	// 	*l_speed = +5.0*K;
	// 	*r_speed = -5.0*K;
	// }
	// // Zone 5
	// else if (gamma > M_PI-(GAMMA+BETA+ALPHA/2) && gamma <= M_PI-(BETA+ALPHA/2)) // point behind on the left
	// {
	// 	*l_speed = -2.0*K;
	// 	*r_speed = -10.0*K;
	// }
	// // Zone 6
	// else if (gamma > M_PI-(BETA+ALPHA/2) && gamma <= M_PI-(ALPHA/2)) // point behind on the right
	// {
	// 	*l_speed = -5.0*K;
	// 	*r_speed = -10.0*K;
	// }
	// // Zone 7
	// else if (gamma > M_PI-(ALPHA/2) && gamma <= M_PI) // point on the right
	// {
	// 	*l_speed = -10.0*K;
	// 	*r_speed = -10.0*K;
	// }
	// // Zone 8
	// else if (gamma > -M_PI && gamma <= -(M_PI-(ALPHA/2))) // point on a bit more the right
	// {
	// 	*l_speed = -10.0*K;
	// 	*r_speed = -10.0*K;
	// }
	// // Zone 9
	// else if (gamma > -(M_PI-(ALPHA/2)) && gamma <= -(M_PI-(BETA+ALPHA/2))) // point a bit on the right
	// {
	// 	*l_speed = -10.0*K;
	// 	*r_speed = -5.0*K;
	// }
	// // Zone 10
	// else if (gamma > -(M_PI-(BETA+ALPHA/2)) && gamma <= -(M_PI-(GAMMA+BETA+ALPHA/2))) // point a bit on the right
	// {
	// 	*l_speed = -10.0*K;
	// 	*r_speed = -2.0*K;
	// }
	// // Zone 11
	// else if (gamma > -(M_PI-(GAMMA+BETA+ALPHA/2)) && gamma <= -M_PI/2)// point a bit on the right
	// {
	// 	*l_speed = -5.0*K;
	// 	*r_speed = +5.0*K;
	// }
	// else if (gamma > -M_PI/2 && gamma <= -(GAMMA+BETA+ALPHA/2)) // point a bit on the right
	// {
	// 	*l_speed = +5.0*K;
	// 	*r_speed = -5.0*K;
	// }
	// else if (gamma > -(GAMMA+BETA+ALPHA/2) && gamma <= -(BETA+ALPHA/2)) // point a bit on the right
	// {
	// 	*l_speed = +10.0*K;
	// 	*r_speed = +2.0*K;
	// }
	// else if (gamma > -(BETA+ALPHA/2) && gamma <= -(ALPHA/2)) // point a bit on the right
	// {
	// 	*l_speed = +10.0*K;
	// 	*r_speed = +5.0*K;
	// }
	// else
	// {
	// 	printf("reach single target failure\n");
	// }
}

NAMESPACE_CLOSE();
