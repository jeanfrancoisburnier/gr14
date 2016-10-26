#include "opp_pos_gr14.h"
#include "init_pos_gr14.h"
#include "useful_gr14.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr14);

#define TOWER_RADIUS 40e-3
#define TOWER2CENTER 83e-3
 
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

	double opp_1_x;
	double opp_1_y;
	double opp_2_x;
	double opp_2_y;

	if (nb_opp == 1)
	{
		if(single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &opp_1_x, &opp_1_y))
		{
			// filter opponent position
			opp_pos->x[0] = first_order_filter(opp_pos->x[0], opp_1_x, 0.3, delta_t);
			opp_pos->y[0] = first_order_filter(opp_pos->y[0], opp_1_y, 0.3, delta_t);
		}

		printf("X1: %f \tY1: %f\n", opp_pos->x[0]*1000.0, opp_pos->y[0]*1000.0);
		// set_plot(rise_1*180/M_PI,"RISE1");
		double distance = sqrt(pow(rob_pos->x-opp_pos->x[0],2)+pow(rob_pos->y-opp_pos->y[0],2));
		set_plot(distance*1000.0," distance [mm]");

	}
	else
	{
		//double distance

		if(single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &opp_1_x, &opp_1_y))
		{
			// filter opponent position
			opp_pos->x[0] = first_order_filter(opp_pos->x[0], opp_1_x, 0.3, delta_t);
			opp_pos->y[0] = first_order_filter(opp_pos->y[0], opp_1_y, 0.3, delta_t);
		}

		if(single_opp_tower(rise_2, fall_2, rob_pos->x, rob_pos->y, rob_pos->theta, &opp_2_x, &opp_2_y))
		{
			// filter opponent position
			opp_pos->x[1] = first_order_filter(opp_pos->x[1], opp_2_x, 0.3, delta_t);
			opp_pos->y[1] = first_order_filter(opp_pos->y[1], opp_2_y, 0.3, delta_t);
		}

		printf("X1: %.2f \t X2: %.2f\n", opp_pos->x[0]*1000.0, opp_pos->x[1]*1000.0);
		printf("Y1: %.2f \t Y2: %.2f\n", opp_pos->y[0]*1000.0, opp_pos->y[1]*1000.0);
		printf("\n");

		double distance1 = sqrt(pow(rob_pos->x-opp_pos->x[0],2)+pow(rob_pos->y-opp_pos->y[0],2));
		double distance2 = sqrt(pow(rob_pos->x-opp_pos->x[1],2)+pow(rob_pos->y-opp_pos->y[1],2));
		set_plot(distance1*1000.0," distance1 [mm]");
		set_plot(distance2*1000.0," distance2 [mm]");
		// set_plot(rise_1*180/M_PI,"RISE1");
		// set_plot(rise_2*180/M_PI,"RISE2");
		//set_plot(opp_pos->x[0],"Opp1 X");
		//set_plot(opp_pos->y[0],"Opp1 Y");

		//set_plot(opp_pos->x[1],"Opp2 X");
		//set_plot(opp_pos->y[1],"Opp2 Y");

	}


	// double distance = sqrt(pow(rob_pos->x-opp_pos->x[0],2)+pow(rob_pos->y-opp_pos->y[0],2));
	// set_plot(distance*1000.0," distance [mm]");
	// printf("Distance: %f\n", distance*1000.0);

	// dont forget to filter!!

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
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp)
{

	double distance = 0.0; // distance between the robot and the opponent

	// deal with special case: angles close to -PI/PI
	if (last_fall < last_rise)
	{
		last_fall += 2*M_PI;
	}

	distance = TOWER_RADIUS/sin((last_fall - last_rise)/2.0);

	*new_x_opp = rob_x + distance*cos((last_fall+last_rise)/2.0 + rob_theta);
	*new_y_opp = rob_y + distance*sin((last_fall+last_rise)/2.0 + rob_theta);

	// printf("Angle: %f\n", (last_fall-last_rise)*180/M_PI);
	// set_plot(last_rise*180.0/M_PI,"last_rise [deg]");
	// set_plot(last_fall*180.0/M_PI,"last_fall [deg]");
	// set_plot(*new_x_opp,"x opp pos [mm]");
	// set_plot(*new_y_opp,"y opp pos [mm]");

	// printf("Distance: %f\n", distance*1000.0);	

	// printf("X coordinate: %f \tY coordinate: %f\n", *new_x_opp*1000.0, *new_y_opp*1000.0);
	if (*new_x_opp < -1.062 || *new_x_opp > 1.062 ||
		*new_y_opp < -1.562 || *new_y_opp > 1.562)
	{
		// printf("Error, computation of opponent position failed!\n");
		return 0;
	}
	else
	{
		return 1;
	}
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
			printf("Too close modafucuzz\n");
			return 1;
		}
	}

	return 0;
}

double compute_distance_to_opp()
{
	double distance = 0.0;

	return distance;
}

NAMESPACE_CLOSE();
