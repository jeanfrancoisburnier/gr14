/*! 
 * \author Group 14
 * \file controller_main_gr14.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr14.h"
#include "namespace_ctrl.h"
#include "init_pos_gr14.h"
#include "odometry_gr14.h"
#include "opp_pos_gr14.h"
#include "speed_regulation_gr14.h"
#include "calibration_gr14.h"
#include "triangulation_gr14.h"
#include "strategy_gr14.h"
#include "kalman_gr14.h"
#include "set_output.h"
#include "path_planning_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/*! \brief initialize controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;

	// robot ID
	cvs->robot_id = inputs->robot_id;

	// robot team
	switch (inputs->robot_id)
	{
		case ROBOT_B: cvs->team_id = TEAM_A; break;
		case ROBOT_R: cvs->team_id = TEAM_A; break;
		case ROBOT_Y: cvs->team_id = TEAM_B; break;
		case ROBOT_W: cvs->team_id = TEAM_B; break;
	
		default:
			printf("Error: unknown robot ID: %d !\n", inputs->robot_id);
			exit(EXIT_FAILURE);
	}

	// number of opponents
	cvs->nb_opp = inputs->nb_opponents;

	// robot initial position
	set_init_position(cvs->robot_id, cvs->rob_pos);
	set_init_position_kalman(cvs->robot_id, cvs->kalman_pos);
	cvs->kalman_pos->last_t = t;
	cvs->rob_pos->last_t = t;

	//init grid
	init_grid();

	// speed regulation
	cvs->sp_reg->last_t = t;


}

/*! \brief controller loop (called every time-step)
 * 
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;
	CtrlOut *outputs;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;

	// time
	t = inputs->t;

	// update the robot odometry
	// update_odometry(cvs);

	// triangulation
	triangulation(cvs);

	//Kalman
	kalman(cvs);

	// opponents position
	opponents_tower(cvs);

	// set_plot(cvs->kalman_pos->x,"x");
	// set_plot(cvs->kalman_pos->y,"y");
	// set_plot(cvs->kalman_pos->theta,"t");

	set_plot(cvs->opp_pos->x[0],"x");
	set_plot(cvs->opp_pos->y[0],"y");

	// set_plot(inputs->r_wheel_speed,"r_wheel_speed");
	// set_plot(inputs->l_wheel_speed,"l_wheel_speed");

	set_output(cvs->kalman_pos->x,"x_r");
	set_output(cvs->kalman_pos->y,"y_r");
	set_output(cvs->kalman_pos->theta,"theta_r");

	// printf("x: %f\t", cvs->kalman_pos->x);
	// printf("y: %f\t", cvs->kalman_pos->y);
	//printf("t: %f\n", cvs->kalman_pos->theta*180/M_PI);

	// printf("x: %f\t", cvs->kalman_pos->x);
	// printf("y: %f\t", cvs->kalman_pos->y);
	// printf("t: %f\n", cvs->kalman_pos->theta*180/M_PI);


	// tower control
	outputs->tower_command = 15.0;

	switch (cvs->main_state)
	{
		// calibration
		case CALIB_STATE:
			calibration(cvs);
			break;

		// wait before match beginning
		case WAIT_INIT_STATE:
			speed_regulation(cvs, 0.0, 0.0);

			if (t > 0.0)
			{
				cvs->main_state = RUN_STATE;
				cvs->strat->main_state = GAME_STATE_COMPUTE_PATH;
			}
			break;

		// during game
		case RUN_STATE:
			main_strategy(cvs);
			if (t > 89.0) // 1 second safety
			{
				cvs->main_state = STOP_END_STATE;
			}
			break;

		// stop at the end of the game
		case STOP_END_STATE:
			speed_regulation(cvs, 0.0, 0.0);

			outputs->flag_release = 1;
			break;

		case NB_MAIN_STATES:
			printf("Error: state NB_MAIN_STATES should not be reached !\n");
			exit(EXIT_FAILURE);
			break;
	
		default:
			printf("Error:unknown state : %d !\n", cvs->main_state);
			exit(EXIT_FAILURE);
	}
}

/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{

}

NAMESPACE_CLOSE();
