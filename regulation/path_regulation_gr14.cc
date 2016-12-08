#include "path_regulation_gr14.h"
#include "useful_gr14.h"
#include "speed_regulation_gr14.h"
#include "init_pos_gr14.h"
#include "strategy_gr14.h"
#include "set_output.h"

NAMESPACE_INIT(ctrlGr14);

#define TAU 0.01
#define RADIUS_TOL 0.0144
#define RADIUS_TOL_NEW 0.05

#define ALPHA 	M_PI/32
#define BETA 	M_PI/16
#define GAMMA 	M_PI/4

static int K = 3;

static size_t i = 0;

static double last_t = 0;

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs, vector<array<float,2> > path)
{
	// Robot current position
	KalmanStruct *kalman_pos = cvs->kalman_pos;

	// Position of the next point to reach
	float x_point = path[i][0];
	float y_point = path[i][1];

	// printf("Going for the point x: %.3f \t y: %.3f\n", x_point, y_point);

	double delta_t = cvs->inputs->t - last_t;
	last_t = cvs->inputs->t;

	// Vector from robot to target
	float vector_x = x_point-kalman_pos->x;
	float vector_y = y_point-kalman_pos->y;
		
	if (i+1 < path.size())
	{
		K = 3;

		if (pow(vector_x,2)+pow(vector_y,2) < RADIUS_TOL) 
		{
			set_output(path[i][0],"x_path");
			set_output(path[i][1],"y_path");
			set_output(path.size(),"path_size");
			i++;
		}
	}
	else if (i+1 == path.size() && pow(vector_x,2)+pow(vector_y,2) < RADIUS_TOL_NEW)
	{
		K = 1;

		if (pow(vector_x,2)+pow(vector_y,2) < 0.005)
		{
			i++;
		}
	}
	// Need to check this ....
	if (i >= path.size())
	{
		// printf("Entering condition 1\n");
		// printf("Color seen: %d\n", cvs->inputs->color_seen);
		i = 0;
		if (cvs->strat->current_target_id <= 7)
		{
			// printf("Entering condition 2\n");

			if (cvs->inputs->color_seen == 4)
			{
				// printf("Entering condition 3\n");
				cvs->outputs->flag_release = 1;
				printf("Target Released!\n");
				// cvs->outputs->flag_release = 0;
			}
			else
			{
				cvs->strat->current_target_id++;
				printf("Target Reached! Robots is at x: %.3f \t y: %.3f\n", kalman_pos->x, kalman_pos->y);
				printf("Deltax: %.3f \t Deltay: %.3f\n", vector_x, vector_y);
				printf("XPoint: %.3f \t YPoint: %.3f\n", x_point, y_point);
			}
			cvs->strat->main_state = GAME_STATE_WAIT;
		}
		else
		{
			cvs->strat->main_state = GAME_STATE_E;
		}
		return;
	}
	// ...Until here

	float beta  = atan2(vector_y,vector_x); // absolute angle of target
	float gamma = limit_angle(beta-kalman_pos->theta); // relative angle of target with robot

	float l_speed = 0;
	float r_speed = 0;

	get_new_speed(gamma, &l_speed, &r_speed);

	// Filter new speed command
	r_speed = first_order_filter(cvs->inputs->r_wheel_speed, r_speed, TAU, delta_t);
	l_speed = first_order_filter(cvs->inputs->l_wheel_speed, l_speed, TAU, delta_t);

	// Set new speed
	speed_regulation(cvs, r_speed, l_speed);
}

void get_new_speed(float gamma, float *l_speed, float *r_speed)
{
	// bool a = 0;
	// bool sign = 0;
	// int b = 0;

	// double speed[4] = {10.0,5.0,2.0,-10.0};

	// if (gamma > 0 && gamma < M_PI/2)
	// {
	// 	a = 0;
	// 	sign = 0;	
	// }
	// else if (gamma > M_PI/2 && gamma < M_PI)
	// {
	// 	a = 0;
	// 	sign = 1;
	// }
	// else if (gamma > -M_PI && gamma < -M_PI/2)
	// {
	// 	a = 1;
	// 	sign = 1;
	// }
	// else if (gamma > -M_PI/2 && gamma < 0)
	// {
	// 	a = 1;
	// 	sign = 0;
	// }
	// else
	// {
	// 	printf("Error\n");
	// 	exit(EXIT_FAILURE);
	// }

	// if (gamma < ALPHA/2)
	// {
	// 	b = 1;
	// }
	// else if (gamma < ALPHA/2+BETA)
	// {
	// 	b = 2;
	// }
	// else if (gamma < ALPHA/2+BETA+GAMMA)
	// {
	// 	b = 3;
	// }
	// else
	// {
	// 	b = 4;
	// }

	// if (a)
	// {
	// 	*l_speed = 10;
	// 	*r_speed = speed[b];
	// }
	// else
	// {
	// 	*l_speed = speed[b];
	// 	*r_speed = 10;
	// }

	// if (sign)
	// {
	// 	*l_speed *= -1.0;
	// 	*r_speed *= -1.0;
	// }

	// Zone 1
	if (gamma > -ALPHA/2 && gamma <= ALPHA/2) // point in front of robot
	{
		*l_speed = +10.0*K;
		*r_speed = +10.0*K;
	}
	// Zone 2
	else if (gamma > ALPHA/2 && gamma <= BETA+ALPHA/2) // point a bit on the left
	{
		*l_speed = +5.0*K;
		*r_speed = +10.0*K;
	}
	// Zone 3
	else if (gamma > BETA+ALPHA/2 && gamma <= GAMMA+BETA+ALPHA/2) // point a bit more on the left
	{
		*l_speed = +2.0*K;
		*r_speed = +10.0*K;
	}
	else if (gamma > GAMMA+BETA+ALPHA/2 && gamma <= M_PI/2)
	{
		*l_speed = -5.0*K;
		*r_speed = +5.0*K;
	}
	// Zone 4
	else if (gamma > M_PI/2 && gamma <= M_PI-(GAMMA+BETA+ALPHA/2)) // point on the left
	{
		*l_speed = +5.0*K;
		*r_speed = -5.0*K;
	}
	// Zone 5
	else if (gamma > M_PI-(GAMMA+BETA+ALPHA/2) && gamma <= M_PI-(BETA+ALPHA/2)) // point behind on the left
	{
		*l_speed = -2.0*K;
		*r_speed = -10.0*K;
	}
	// Zone 6
	else if (gamma > M_PI-(BETA+ALPHA/2) && gamma <= M_PI-(ALPHA/2)) // point behind on the right
	{
		*l_speed = -5.0*K;
		*r_speed = -10.0*K;
	}
	// Zone 7
	else if (gamma > M_PI-(ALPHA/2) && gamma <= M_PI) // point on the right
	{
		*l_speed = -10.0*K;
		*r_speed = -10.0*K;
	}
	// Zone 8
	else if (gamma > -M_PI && gamma <= -(M_PI-(ALPHA/2))) // point on a bit more the right
	{
		*l_speed = -10.0*K;
		*r_speed = -10.0*K;
	}
	// Zone 9
	else if (gamma > -(M_PI-(ALPHA/2)) && gamma <= -(M_PI-(BETA+ALPHA/2))) // point a bit on the right
	{
		*l_speed = -10.0*K;
		*r_speed = -5.0*K;
	}
	// Zone 10
	else if (gamma > -(M_PI-(BETA+ALPHA/2)) && gamma <= -(M_PI-(GAMMA+BETA+ALPHA/2))) // point a bit on the right
	{
		*l_speed = -10.0*K;
		*r_speed = -2.0*K;
	}
	// Zone 11
	else if (gamma > -(M_PI-(GAMMA+BETA+ALPHA/2)) && gamma <= -M_PI/2)// point a bit on the right
	{
		*l_speed = -5.0*K;
		*r_speed = +5.0*K;
	}
	else if (gamma > -M_PI/2 && gamma <= -(GAMMA+BETA+ALPHA/2)) // point a bit on the right
	{
		*l_speed = +5.0*K;
		*r_speed = -5.0*K;
	}
	else if (gamma > -(GAMMA+BETA+ALPHA/2) && gamma <= -(BETA+ALPHA/2)) // point a bit on the right
	{
		*l_speed = +10.0*K;
		*r_speed = +2.0*K;
	}
	else if (gamma > -(BETA+ALPHA/2) && gamma <= -(ALPHA/2)) // point a bit on the right
	{
		*l_speed = +10.0*K;
		*r_speed = +5.0*K;
	}
	else
	{
		printf("reach single target failure\n");
	}
}

void reset_current_point_id(void)
{
	i = 0;
	return;
}

NAMESPACE_CLOSE();
