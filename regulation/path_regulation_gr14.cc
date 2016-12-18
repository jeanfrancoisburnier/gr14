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
#define RADIUS_TOL 0.02//0.0144
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
	double x_point = path[i][0];
	double y_point = path[i][1];

	double delta_t = cvs->inputs->t - last_t;
	last_t = cvs->inputs->t;

	// Vector from robot to target
	double vector_x = x_point-kalman_pos->x;
	double vector_y = y_point-kalman_pos->y;

	if (i < path.size() - 1)
	{
		K = 5;
		if(test_opponent_too_close(cvs, FIRST_SECURITY_DISTANCE))
		{
			if(test_opponent_too_close(cvs, LAST_SECURITY_DISTANCE))
			{
				K = 0;
			}
			else
			{
				K = 2;
			}
		}

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

		if(test_opponent_too_close(cvs, FIRST_SECURITY_DISTANCE))
		{
			if(test_opponent_too_close(cvs, LAST_SECURITY_DISTANCE))
			{
				K = 0;
			}
			else
			{
				K = 1;
			}
		}

		if (pow(vector_x,2)+pow(vector_y,2) < RADIUS_TOL_TAR)
		{
			if (strat->status == STRAT_TARGET)
			{
				strat->main_state = GAME_STATE_WAIT;
			}
			else if (strat->status == STRAT_SCORING)
			{
				strat->current_point_id = 0;
				cvs->outputs->flag_release = 1;

				strat->target[strat->carrying_target_id[0]].status = TARGET_WON;
				strat->remaining_targets--;
				if (strat->prev_nb_target_carrying > 1)
				{
					strat->target[strat->carrying_target_id[1]].status = TARGET_WON;
					strat->remaining_targets--;

				}
				strat->carrying_target_id[0] = -1;
				strat->carrying_target_id[1] = -1;
				strat->prev_nb_target_carrying = 0;
				strat->main_state = GAME_STATE_COMPUTE_PATH;
			}
			return;
		}
	}

	double beta  = atan2(vector_y,vector_x); // absolute angle of target
	double gamma = limit_angle(beta-kalman_pos->theta); // relative angle of target with robot

	double l_speed = 0;
	double r_speed = 0;

	get_new_speed(gamma, &l_speed, &r_speed);

	// Filter new speed command
	r_speed = first_order_filter(cvs->inputs->r_wheel_speed, r_speed, TAU, delta_t);
	l_speed = first_order_filter(cvs->inputs->l_wheel_speed, l_speed, TAU, delta_t);
	// Set new speed
	speed_regulation(cvs, r_speed, l_speed);
}

void get_new_speed(double gamma, double *l_speed, double *r_speed)
{
	int quadrant = 0;
	int b = 0;

	double speed_a[4] = {+10.0,+5.0,+2.0,-5.0};
	double speed_b[4] = {+10.0,+10.0,+10.0,+5.0};

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
		*l_speed = speed_b[b];
		*r_speed = speed_a[b];
	}
	else
	{
		*l_speed = speed_a[b];
		*r_speed = speed_b[b];
	}

	if (quadrant & 1)
	{
		*l_speed *= -1.0;
		*r_speed *= -1.0;
	}

	*l_speed *= K;
	*r_speed *= K;

}

NAMESPACE_CLOSE();
