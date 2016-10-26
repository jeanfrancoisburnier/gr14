#include "triangulation_gr14.h"
#include "useful_gr14.h"
#include "init_pos_gr14.h"
#include <math.h>
#define declage_tower 0.083

#define PI 3,1416

NAMESPACE_INIT(ctrlGr14);

/*! \brief set the fixed beacons positions, depending on the team
 * 
 * \param[in] team_id ID of the team ('TEAM_A' or 'TEAM_B')
 * \param[out] x_beac_1 first beacon x position [m]
 * \param[out] y_beac_1 first beacon y position [m]
 * \param[out] x_beac_2 second beacon x position [m]
 * \param[out] y_beac_2 second beacon y position [m]
 * \param[out] x_beac_3 third beacon x position [m]
 * \param[out] y_beac_3 third beacon y position [m]
 *
 * This function can be adapted, depending on the map.
 */
void fixed_beacon_positions(int team_id, double *x_beac_1, double *y_beac_1,
	double *x_beac_2, double *y_beac_2, double *x_beac_3, double *y_beac_3)
{
	switch (team_id)
	{
		case TEAM_A: //blue team
			*x_beac_1 = 1.062;
			*y_beac_1 = 1.562;

			*x_beac_2 = -1.062;
			*y_beac_2 = 1.562;

			*x_beac_3 = 0.0;
			*y_beac_3 = -1.562;
			break;

		case TEAM_B:
			*x_beac_1 = 1.062;
			*y_beac_1 = -1.562;

			*x_beac_2 = -1.062;
			*y_beac_2 = -1.562;

			*x_beac_3 = 0.0;
			*y_beac_3 = -1.562;
			break;
	
		default:
			printf("Error unknown team ID (%d) !\n", team_id);
			exit(EXIT_FAILURE);
	}
}

/*! \brief get the index of the best angle prediction
 * 
 * \param[in] alpha_predicted angle to reach [rad]
 * \param[in] alpha_a angle computed for A [rad]
 * \param[in] alpha_b angle computed for B [rad]
 * \param[in] alpha_c angle computed for C [rad]
 * \return best index (0, 1, or 2)
 */
int index_predicted(double alpha_predicted, double alpha_a, double alpha_b, double alpha_c)
{
	double pred_err_a, pred_err_b, pred_err_c;

	pred_err_a = fabs(limit_angle(alpha_a - alpha_predicted));
	pred_err_b = fabs(limit_angle(alpha_b - alpha_predicted));
	pred_err_c = fabs(limit_angle(alpha_c - alpha_predicted));

	return (pred_err_a < pred_err_b) ? ((pred_err_a < pred_err_c) ? 0 : 2) : ((pred_err_b < pred_err_c) ? 1 : 2);
}

/*! \brief triangulation main algorithm
 * 
 * \param[in] cvs controller main structure
 *
 * computation found here: http://www.telecom.ulg.ac.be/triangulation/
 */
void triangulation(CtrlStruct *cvs)
{
	// variables declaration
	RobotPosition *pos_tri, *rob_pos;
	CtrlIn *inputs;

	int alpha_1_index, alpha_2_index, alpha_3_index;
	int rise_index_1, rise_index_2, rise_index_3;
	int fall_index_1, fall_index_2, fall_index_3;

	double alpha_a, alpha_b, alpha_c;
	double alpha_1, alpha_2, alpha_3;
	double alpha_1_predicted, alpha_2_predicted, alpha_3_predicted;
	double x_beac_1, y_beac_1, x_beac_2, y_beac_2, x_beac_3, y_beac_3;
	
	//variables needed for the ToTal algorithm
	/*double xm_beac_1, ym_beac_1, xm_beac_3, ym_beac_3;
	double T12, T23, T31;
	double x12_p, x23_p, x31_p, y12_p, y23_p, y31_p, k31_p;
	double D;*/

	// variables initialization
	pos_tri = cvs->triang_pos;
	rob_pos = cvs->rob_pos;
	inputs  = cvs->inputs;
	// safety
	if ((inputs->rising_index_fixed < 0) || (inputs->falling_index_fixed < 0))
	{
		return;
	}

	// known positions of the beacons
	fixed_beacon_positions(cvs->team_id, &x_beac_1, &y_beac_1, &x_beac_2, &y_beac_2, &x_beac_3, &y_beac_3);	

	// indexes fot the angles detection
	rise_index_1 = inputs->rising_index_fixed;
	rise_index_2 = (rise_index_1 - 1 < 0) ? NB_STORE_EDGE-1 : rise_index_1 - 1;
	rise_index_3 = (rise_index_2 - 1 < 0) ? NB_STORE_EDGE-1 : rise_index_2 - 1;

	fall_index_1 = inputs->falling_index_fixed;
	fall_index_2 = (fall_index_1 - 1 < 0) ? NB_STORE_EDGE-1 : fall_index_1 - 1;
	fall_index_3 = (fall_index_2 - 1 < 0) ? NB_STORE_EDGE-1 : fall_index_2 - 1;

	// beacons angles measured with the laser (to compute)
	alpha_a = 0.0;
	alpha_b = 0.0;
	alpha_c = 0.0;

	// beacons angles predicted thanks to odometry measurements (to compute)
	alpha_1_predicted = predicted_angle(rob_pos->x,rob_pos->y,x_beac_1,y_beac_1,rob_pos->theta,declage_tower);
	alpha_2_predicted = predicted_angle(rob_pos->x,rob_pos->y,x_beac_2,y_beac_2,rob_pos->theta,declage_tower);
	alpha_3_predicted = predicted_angle(rob_pos->x,rob_pos->y,x_beac_3,y_beac_3,rob_pos->theta,declage_tower);

	// indexes of each beacon
	alpha_1_index = index_predicted(alpha_1_predicted, alpha_a, alpha_b, alpha_c);
	alpha_2_index = index_predicted(alpha_2_predicted, alpha_a, alpha_b, alpha_c);
	alpha_3_index = index_predicted(alpha_3_predicted, alpha_a, alpha_b, alpha_c);

	// safety
	if ((alpha_1_index == alpha_2_index) || (alpha_1_index == alpha_3_index) || (alpha_2_index == alpha_3_index))
	{
		return;
	}

	// angle of the first beacon
	switch (alpha_1_index)
	{
		case 0: alpha_1 = alpha_a; break;
		case 1: alpha_1 = alpha_b; break;
		case 2: alpha_1 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_1_index);
			exit(EXIT_FAILURE);
	}

	// angle of the second beacon
	switch (alpha_2_index)
	{
		case 0: alpha_2 = alpha_a; break;
		case 1: alpha_2 = alpha_b; break;
		case 2: alpha_2 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_2_index);
			exit(EXIT_FAILURE);
	}

	// angle of the third beacon
	switch (alpha_3_index)
	{
		case 0: alpha_3 = alpha_a; break;
		case 1: alpha_3 = alpha_b; break;
		case 2: alpha_3 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_3_index);
			exit(EXIT_FAILURE);
	}
	

	/* ----- triangulation computation start ----- //
	* ToTal algorithm : http://www.telecom.ulg.ac.be/triangulation/
 	* Version with mathematical approximation of the limit for the pseudosingularities
	*/
	float cot_12 = 1/tan( alpha_2 - alpha_1 ) ;
	float cot_23 = 1/tan( alpha_3 - alpha_2 ) ;
	cot_12 = adjust_value_to_bounds( cot_12 , COT_MAX ) ;
	cot_23 = adjust_value_to_bounds( cot_23 , COT_MAX ) ;
	float cot_31 = ( 1.0 - cot_12 * cot_23 ) / ( cot_12 + cot_23 ) ;
	cot_31 = adjust_value_to_bounds( cot_31 , COT_MAX ) ;
	
	float x1_ = x_beac_1 - x_beac_2 , y1_ = y_beac_1 - y_beac_2 , x3_ = x_beac_3 - x_beac_2 , y3_ = y_beac_3 - y_beac_2 ;

	float c12x = x1_ + cot_12 * y1_ ;
	float c12y = y1_ - cot_12 * x1_ ;

	float c23x = x3_ - cot_23 * y3_ ;
	float c23y = y3_ + cot_23 * x3_ ;

	float c31x = (x3_ + x1_) + cot_31 * (y3_ - y1_) ;
	float c31y = (y3_ + y1_) - cot_31 * (x3_ - x1_) ;
	float k31 = (x3_ * x1_) + (y3_ * y1_) + cot_31 * ( (y3_ * x1_) - (x3_ * y1_) ) ;
  
  	float D = (c12x - c23x) * (c23y - c31y) - (c23x - c31x) * (c12y - c23y) ;
  	float invD = 1.0 / D ;
  	float K = k31 / invD ;
  
  	//Position of the Robot
	pos_tri->x = K * (c12y - c23y) + x2 ;
	pos_tri->y = K * (c23x - c12x) + y2 ;
	

	//Orientation of the Robot //**********************Fait au cas par cas, il faudrait vérifier si pas déjà un algo existant*****
	float x_abs = abs(pos_tri->x);
	float xr2 = abs(x_beac_2 - pos_tri->x); // distance robot -> beacoin 2 en x
	float xr1 = abs(x_beac_1 - pos_tri->x);// distance robot -> beacoin 1 en x
	float yr1 = abs(y_beac_1 - pos_tri->y);// distance robot -> beatcoin 1 en y
	float yr3 = abs(y_beac_3 - pos_tri->y);// distance robot -> beatcoin 3 en y


	if((alpha_1 < 0) && (alpha_2 >= 0) && (alpha_3 < 0)
	{
		pos_tri->theta = arctan(yr1 / xr1) - alpha_1;
	}
	else if((alpha_1 < 0) && (alpha_2 < 0) && (alpha_3 >= 0)
	{
		pos_tri->theta = arctan(xr2/yr1) - alpha_2 + PI/2;
	}
	else if((alpha_1 < 0) && (alpha_2 >= 0) && (alpha_3 >= 0)
	{
		pos_tri->theta = arctan(yr1/xr1) - alpha_1;
	}
	else if((alpha_1 >= 0) && (alpha_2 < 0) && (alpha_3 < 0)
	{
		pos_tri->theta = - arctan(x_abs / yr3) - alpha_3 - PI/2;
	}
	else if((alpha_1 >= 0) && (alpha_2 >= 0) && (alpha_3 >= 0)
	{
		pos_tri->theta = arctan(yr1 / xr3) - alpha_1;
	}
	else if((alpha_1 < 0) && (alpha_2 < 0) && (alpha_3 < 0)
	{
		pos_tri->theta = - arctan(x_abs / yr3) - alpha_3 - PI/2;
	}
	else if((alpha_1 >= 0) && (alpha_2 >= 0) && (alpha_3 < 0)
	{
		pos_tri->theta = - arctan(yr3 / x_abs) - alpha_3;
	}
	else//else if((alpha_1 >= 0) && (alpha_2 < 0) && (alpha_3 >= 0)
	{
		pos_tri->theta = arctan(x_abs / yr3) - alpha_3 - PI/2;
	}


	// ----- triangulation computation end ----- //
}

double predicted_angle(double x_r,double y_r,double x_b,double y_b,double alpha,double d){
	double theta;
	theta = atan((x_b-(x_r+cos(alpha)*d))/(y_b-(y_r+sin(alpha)*d)))-alpha;
return theta;
}

NAMESPACE_CLOSE();
