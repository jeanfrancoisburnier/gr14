#include "init_pos_gr14.h"
#include <math.h>


#define SIGMA_X 7.9e-6
#define SIGMA_Y 7.9e-6
#define SIGMA_THETA 1e-6
#define SIGMA_Q 2e-9

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
			rob_pos->x = 0.82;
			rob_pos->y = 1.4;
			rob_pos->theta = - 3.14156/2;
			break;

		case ROBOT_Y: // yellow robot
			rob_pos->x = 0.67;
			rob_pos->y = - 1.15;
			rob_pos->theta = 3.14156/2;
			break;

		case ROBOT_W: //  white robot
			rob_pos->x = 0.82;
			rob_pos->y = - 1.4;
			rob_pos->theta = 3.14156/2;
			break;

		default:
			printf("Error: unknown robot ID: %d !\n", robot_id);
			exit(EXIT_FAILURE);
	}
}

/*! \brief set the initial robot position for the kalman filter
 *
 * \param[in] robot_id robot ID
 * \param[out] rob_pos robot position structure
 * \parma[out] The values for the matrix Q,R and the initial value of P_k(which is Q)
 *
 * Adapt these initial positions, depending on the game map.
 */
void set_init_position_kalman(int robot_id, KalmanStruct *kalman_pos)
{
	//initialise the matrix to 0 and then set the diagonal
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
			kalman_pos->x = 0.82;
			kalman_pos->y = 1.4;
			kalman_pos->theta = - 3.14156/2;
			break;

		case ROBOT_Y: // yellow robot
			kalman_pos->x = 0.67;
			kalman_pos->y = - 1.15;
			kalman_pos->theta = 3.14156/2;
			break;

		case ROBOT_W: //  white robot
			kalman_pos->x = 0.82;
			kalman_pos->y = - 1.4;
			kalman_pos->theta = 3.14156/2;
			break;

		default:
			printf("Error: unknown robot ID: %d !\n", robot_id);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
