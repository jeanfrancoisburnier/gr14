#include "opp_pos_gr14.h"
#include "init_pos_gr14.h"
#include "useful_gr14.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr14);

#define TOWER_RADIUS 40e-3
#define TOWER2CENTER 83e-3
#define TAU 0.3
 
/*! \brief compute the opponents position using the tower
 * 
 * \param[in,out] cvs controller main structure
 */
void opponents_tower(CtrlStruct *cvs)
{
	// variables declaration
	int nb_opp;
	int rise_index_1, rise_index_2, fall_index_1, fall_index_2;

	double delta_t;
	double rise_1, rise_2, fall_1, fall_2;

	CtrlIn *inputs;
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;

	nb_opp = opp_pos->nb_opp;

	// no opponent
	if (!nb_opp)
	{
		return;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	// low pass filter time increment ('delta_t' is the last argument of the 'first_order_filter' function)
	delta_t = inputs->t - opp_pos->last_t;
	opp_pos->last_t = inputs->t;

	// indexes
	rise_index_1 = inputs->rising_index;
	fall_index_1 = inputs->falling_index;

	// rise and fall angles of the first opponent
	rise_1 = inputs->last_rising[rise_index_1];
	fall_1 = inputs->last_falling[fall_index_1];

	// rise and fall angles of the second opponent
	if (nb_opp == 2)
	{
		rise_index_2 = (rise_index_1-1 < 0) ? NB_STORE_EDGE-1 : rise_index_1-1;
		fall_index_2 = (fall_index_1-1 < 0) ? NB_STORE_EDGE-1 : fall_index_1-1;

		rise_2 = inputs->last_rising[rise_index_2];
		fall_2 = inputs->last_falling[fall_index_2];
	}

	// ----- opponents position computation start ----- //

	// Opponent a position
	double opp_a_x;
	double opp_a_y;

	// Opponent b position
	double opp_b_x;
	double opp_b_y;

	double delta_pos_1;
	double delta_pos_2;

	if (nb_opp == 1)
	{
		opp_a_x = opp_pos->x[0];
		opp_a_y = opp_pos->y[0];

		// Get new position of opponent a
		single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &opp_a_x, &opp_a_y);

		// filter opponent position
		opp_pos->x[0] = first_order_filter(opp_pos->x[0], opp_a_x, TAU, delta_t);
		opp_pos->y[0] = first_order_filter(opp_pos->y[0], opp_a_y, TAU, delta_t);

	}
	else
	{

		// Get new position of opponent a and b
		single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &opp_a_x, &opp_a_y);
		single_opp_tower(rise_2, fall_2, rob_pos->x, rob_pos->y, rob_pos->theta, &opp_b_x, &opp_b_y);

		// Compute...
		delta_pos_1 = pow(opp_a_x - opp_pos->x[0],2) + pow(opp_a_y - opp_pos->y[0],2);
		delta_pos_2 = pow(opp_b_x - opp_pos->x[0],2) + pow(opp_b_y - opp_pos->y[0],2);

		if(delta_pos_1 < delta_pos_2)
		{
			// filter opponent position
			opp_pos->x[0] = first_order_filter(opp_pos->x[0], opp_a_x, TAU, delta_t);
			opp_pos->y[0] = first_order_filter(opp_pos->y[0], opp_a_y, TAU, delta_t);

			opp_pos->x[1] = first_order_filter(opp_pos->x[1], opp_b_x, TAU, delta_t);
			opp_pos->y[1] = first_order_filter(opp_pos->y[1], opp_b_y, TAU, delta_t);

		}
		else
		{
			// filter opponent position
			opp_pos->x[0] = first_order_filter(opp_pos->x[0], opp_b_x, TAU, delta_t);
			opp_pos->y[0] = first_order_filter(opp_pos->y[0], opp_b_y, TAU, delta_t);

			opp_pos->x[1] = first_order_filter(opp_pos->x[1], opp_a_x, TAU, delta_t);
			opp_pos->y[1] = first_order_filter(opp_pos->y[1], opp_a_y, TAU, delta_t);

		}

		// double distance1 = sqrt(pow(rob_pos->x-opp_pos->x[0],2)+pow(rob_pos->y-opp_pos->y[0],2));
		// double distance2 = sqrt(pow(rob_pos->x-opp_pos->x[1],2)+pow(rob_pos->y-opp_pos->y[1],2));

		// set_plot(distance1*1000.0," distance1 [mm]");
		// set_plot(distance2*1000.0," distance2 [mm]");

	}
	// ----- opponents position computation end ----- //
}

/*! \brief compute a single opponent position
 * 
 * \param[in] last_rise last rise relative angle [rad]
 * \param[in] last_fall last fall relative angle [rad]
 * \param[in] rob_x robot x position [m]
 * \param[in] rob_y robot y position [m]
 * \param[in] rob_theta robot orientation [rad]
 * \param[out] new_x_opp new known x opponent position
 * \param[out] new_y_opp new known y opponent position
 * \return 1 if computation successful, 0 otherwise
 */
void single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp)
{

	double distance = 0.0; // distance between the robot and the opponent

	// deal with special case: angles close to -PI/PI
	if (last_fall < last_rise)
	{
		last_fall += 2*M_PI;
	}

	distance = TOWER_RADIUS/sin((last_fall - last_rise)/2.0);

	if (distance == INFINITY)
	{
		return;
	}

	*new_x_opp = rob_x + distance*cos((last_fall+last_rise)/2.0 + rob_theta);
	*new_y_opp = rob_y + distance*sin((last_fall+last_rise)/2.0 + rob_theta);
	
	return;
}

/*! \brief check if there is an opponent in front of the robot
 * 
 * \param[in] cvs controller main structure
 * \return 1 if opponent robot in front of the current robot
 */
int check_opp_front(CtrlStruct *cvs)
{
	// variables declaration
	int i, nb_opp;

	OpponentsPosition *opp_pos;
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;
	nb_opp = opp_pos->nb_opp;

	// no opponent
	if (!nb_opp)
	{
		return 0;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	for(i=0; i<nb_opp; i++)
	{
		if (pow((opp_pos->x[i] - rob_pos->x + TOWER2CENTER*cos(rob_pos->theta)),2) +
			pow((opp_pos->y[i] - rob_pos->y + TOWER2CENTER*sin(rob_pos->theta)),2) < 0.3 )
		{
			// printf("Too close modafucuzz\n");
			return 1;
		}
	}

	return 0;
}

NAMESPACE_CLOSE();
