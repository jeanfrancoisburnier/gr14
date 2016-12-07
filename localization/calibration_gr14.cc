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
	KalmanStruct *kalman_pos;

	// variables initialization
	inputs = cvs->inputs;
	calib  = cvs->calib;

	kalman_pos = cvs->kalman_pos;
	
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

		case CALIB_STATE_A: // move backward until contact with wall
			speed_regulation(cvs, -5.0, -5.0);

			// go to state B after 5 seconds
			if (t - calib->t_flag > 2.0 || (inputs->u_switch[0] && inputs->u_switch[1]))
			{
				calib->flag = CALIB_STATE_B;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_B: // move backward until pressed against wall -> initializes y coordinate and (theta)

			// go to state C after 0.5 second
			if (t - calib->t_flag > 0.5)
			{
				calib->flag = CALIB_STATE_C;

				calib->t_flag = t;

				// Do not forget to take into that the robot could start on yellow side
				kalman_pos->y = 1.44;
				kalman_pos->theta = -M_PI/2;

			}
			break;

		case CALIB_STATE_C: // move forward
			speed_regulation(cvs, 5.0, 5.0);

			// go to state D after 1 second
			if (t - calib->t_flag > 1.0)
			{
				calib->flag = CALIB_STATE_D;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_D: // rotation of -90°
			speed_regulation(cvs, -2.356, +2.356);

			// go to state E after 2.5 seconds
			if (t - calib->t_flag > 2.5)
			{
				calib->flag = CALIB_STATE_E;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_E: // move backward until contact with wall
			speed_regulation(cvs, -5.0, -5.0);

			 // go to state F after 2 seconds
			if (t - calib->t_flag > 2.0 || (inputs->u_switch[0] && inputs->u_switch[1]))
			{
				calib->flag = CALIB_STATE_F;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_F: // move backward until pressed against wall -> initializes x coordinate and theta

			// go to state G after 0.5 second
			if (t - calib->t_flag > 0.5)
			{
				calib->flag = CALIB_STATE_G;

				calib->t_flag = t;

				// Do not forget to take into that the robot could start on yellow side
				kalman_pos->x = 0.94;
				kalman_pos->theta = M_PI;
			}
			break;

		case CALIB_STATE_G: // move forward
			speed_regulation(cvs, 5.0, 5.0);

			// go to state H after 2 seconds
			if (t - calib->t_flag > 1.5)
			{
				calib->flag = CALIB_STATE_H;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_H: // rotation of 90°
			speed_regulation(cvs, +2.356, -2.356);

			// go to final state after 2.5 seconds
			if (t - calib->t_flag > 2.5)
			{
				calib->flag = CALIB_FINISH;

				calib->t_flag = t;
			}
			break;

		case CALIB_FINISH: // wait before the match is starting
			speed_regulation(cvs, 0.0, 0.0);
			cvs->main_state = WAIT_INIT_STATE;
			break;
	
		default:
			printf("Error: unknown state : %d !\n", calib->flag);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
