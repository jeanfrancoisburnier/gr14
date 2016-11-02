#include "init_pos_gr14.h"
#include <math.h>

#define SIGMA_X 0.01
#define SIGMA_Y 0.01
#define SIGMA_THETA 0.01

#define SIGMA_Q 0.01

NAMESPACE_INIT(ctrlGr14);

/*! \brief set the initial robot position
 * 
 * \param[in] robot_id robot ID
 * \param[out] rob_pos robot position structure
 *
 * Adapt these initial positions, depending on the game map.
 */
void set_init_position(int robot_id, RobotPosition *rob_pos)
{
	switch (robot_id)
	{
		case ROBOT_B: // blue robot
			rob_pos->x = 0.67;
			rob_pos->y = 1.15;
			rob_pos->theta = - 3.14156/2;
			break;

		case ROBOT_R: // red robot
			rob_pos->x = 0.0;
			rob_pos->y = 0.0;
			rob_pos->theta = 0.0;
			break;

		case ROBOT_Y: // yellow robot
			rob_pos->x = 0.0;
			rob_pos->y = 0.0;
			rob_pos->theta = 0.0;
			break;

		case ROBOT_W: //  white robot
			rob_pos->x = 0.0;
			rob_pos->y = 0.0;
			rob_pos->theta = 0.0;
			break;
	
		default:
			printf("Error: unknown robot ID: %d !\n", robot_id);
			exit(EXIT_FAILURE);
	}		
}


void set_init_position_kalman(int robot_id, KalmanStruct *kalman_pos)
{	
	int i,j;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			kalman_pos->Q[i][j] = 0;
		}
	}

	kalman_pos->Q[0][0] = SIGMA_Q;
	kalman_pos->Q[1][1] = SIGMA_Q;
	kalman_pos->Q[2][2] = SIGMA_Q;
	
	
	for(i = 0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			kalman_pos->R[i][j] = 0;
		}
	}
	
	kalman_pos->R[0][0] = SIGMA_X;
	kalman_pos->R[1][1] = SIGMA_Y;
	kalman_pos->R[2][2] = SIGMA_THETA;
	
	
	for(i = 0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			kalman_pos->P_k[i][j] = kalman_pos->Q[i][j];
		}
	}
	

	switch (robot_id)
	{
		case ROBOT_B: // blue robot
			kalman_pos->x = 0.67;
			kalman_pos->y = 1.15;
			kalman_pos->theta = - 3.14156/2;
			break;

		case ROBOT_R: // red robot
			kalman_pos->x = 0.0;
			kalman_pos->y = 0.0;
			kalman_pos->theta = 0.0;
			break;

		case ROBOT_Y: // yellow robot
			kalman_pos->x = 0.0;
			kalman_pos->y = 0.0;
			kalman_pos->theta = 0.0;
			break;

		case ROBOT_W: //  white robot
			kalman_pos->x = 0.0;
			kalman_pos->y = 0.0;
			kalman_pos->theta = 0.0;
			break;
	
		default:
			printf("Error: unknown robot ID: %d !\n", robot_id);
			exit(EXIT_FAILURE);
	}		
}

NAMESPACE_CLOSE();
