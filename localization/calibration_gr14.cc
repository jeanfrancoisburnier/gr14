#include "calibration_gr14.h"
#include "speed_regulation_gr14.h"
#include "odometry_gr14.h"
#include "useful_gr14.h"
#include "init_pos_gr14.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr14);

#define DEG_TO_RAD (M_PI/180.0) ///< convertion from degrees to radians

// calibration states
enum {CALIB_START, CALIB_STATE_A, CALIB_STATE_B, CALIB_STATE_C, CALIB_STATE_D, CALIB_STATE_E, CALIB_STATE_F, CALIB_STATE_G, CALIB_STATE_H, CALIB_FINISH};

/*! \brief calibration of the robot to calibrate its position
 * 
 * \param[in,out] cvs controller main structure
 * 
 * This FSM can be adapted, depending on the map and on the robots initial position.
 */
void calibration(CtrlStruct *cvs)
{
	// variables declaration
	int team_id;
	double t;

	CtrlIn *inputs;
	RobotCalibration *calib;
	RobotPosition *rob_pos;

	// variables initialization
	inputs = cvs->inputs;
	calib  = cvs->calib;

	rob_pos = cvs->rob_pos;
	
	t = inputs->t;
	team_id = cvs->team_id;

	// finite state machine (FSM)
	switch (calib->flag)
	{
		case CALIB_START: // start calibration
			speed_regulation(cvs, 0.0, 0.0);

			calib->flag = CALIB_STATE_A; // directly go to state A
			calib->t_flag = t;
			break;

		case CALIB_STATE_A: // state A
			speed_regulation(cvs, -4.0, -4.0);

			// go to state B after 5 seconds
			if (t - calib->t_flag > 5.0 || (cvs->inputs->u_switch[0] && cvs->inputs->u_switch[1]))
			{
				calib->flag = CALIB_STATE_B;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_B: // state B - bien collé au mur
			// speed_regulation(cvs, 5.0, 5.0);

			// go to state C after 0.5 seconds
			if (t - calib->t_flag > 1.0)
			{
				calib->flag = CALIB_STATE_C;

				calib->t_flag = t;

				// should send something (position x or y)/////////////////////////
			}
			break;

		case CALIB_STATE_C: // state C
			speed_regulation(cvs, 5.0, 5.0);

			// go to final state after 2 seconds
			if (t - calib->t_flag > 1.0)
			{
				calib->flag = CALIB_STATE_D;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_D: // state D
			speed_regulation(cvs, -2.356, +2.356);

			// go to final state after 5 seconds
			if (t - calib->t_flag > 2.5)
			{
				calib->flag = CALIB_STATE_E;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_E: // state E
			speed_regulation(cvs, -4.0, -4.0);

			// go to final state after 2 seconds
			if (t - calib->t_flag > 5.0 || (cvs->inputs->u_switch[0] && cvs->inputs->u_switch[1]))
			{
				calib->flag = CALIB_STATE_F;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_F: // state F - bien collé au mur
			// speed_regulation(cvs, 5.0, 5.0);

			// go to state C after 0.5 seconds
			if (t - calib->t_flag > 1.0)
			{
				calib->flag = CALIB_STATE_G;

				calib->t_flag = t;

				// should send something (position x or y)/////////////////////////
			}
			break;

		case CALIB_STATE_G: // state G
			speed_regulation(cvs, 4.0, 4.0);

			// go to final state after 2 seconds
			if (t - calib->t_flag > 2.0)
			{
				calib->flag = CALIB_STATE_H;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_H: // state H
			speed_regulation(cvs, +2.356, -2.356);

			// go to final state after 2 seconds
			if (t - calib->t_flag > 2.5)
			{
				calib->flag = CALIB_FINISH;

				calib->t_flag = t;
			}
			break;

		case CALIB_FINISH: // wait before the match is starting
			speed_regulation(cvs, 50.0, 50.0);
			break;
	
		default:
			printf("Error: unknown state : %d !\n", calib->flag);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
