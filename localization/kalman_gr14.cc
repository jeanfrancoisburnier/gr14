#include "kalman_gr14.h"
#include "odometry_gr14.h"
#include "triangulation_gr14.h"
#include "useful_gr14.h"
#include <math.h>
#include <cmath>


#define WHEEL_SEP 0.225
#define WHEEL_RAD 0.030
#define PI 3.1416

NAMESPACE_INIT(ctrlGr14);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void kalman(CtrlStruct *cvs)
{
	// variable declaration

	//RobotPosition *rob_pos;
	RobotPosition *triang_pos;
	KalmanStruct *kalman_pos;
	CtrlIn *inputs;

	//rob_pos = cvs->rob_pos;
	inputs  = cvs->inputs;
	triang_pos = cvs->triang_pos;
	kalman_pos = cvs->kalman_pos;

	double z_k[3] = {(*triang_pos).x,(*triang_pos).y,(*triang_pos).theta};
	double x_hat[3] = {(*kalman_pos).x,(*kalman_pos).y,(*kalman_pos).theta};
	/*printf("x_hat;");
	print_vect(x_hat);*/
	double P_hat[3][3];
	/*printf("P_hat:");
	print_mat(P_hat);*/
	copy_mat((*kalman_pos).P_k,P_hat);
	double A_k[3][3];
	double A_k_trans[3][3] = { 0 };
	double B_k[3][3];
	double S_k[3][3];
	double K_k[3][3];
	double u_k[3];
	double I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	double err_k[3];
	double dSl,dSr,dS,d_theta;
	double dt;
	double r_sp,l_sp;
	double result_vect[3];
	double result_matrix[3][3];//result vector and matrix used as intermediates in the algorithm

	// variables initialization

	//rob_pos = cvs->rob_pos;

	//------compute dSl, dSr, d_theta and dS for the matrices A and B--------//
	r_sp = wheel_speed_meter(inputs->r_wheel_speed,WHEEL_RAD); // right wheel speed
	l_sp = wheel_speed_meter(inputs->l_wheel_speed,WHEEL_RAD); // left wheel speed

	// time
	dt = inputs->t - kalman_pos->last_t; // time increment since last call

	// safety
	if (dt <= 0.0)
	{
		return;
	}

    dSl = dt * l_sp ;
    dSr = dt * r_sp ;
    
    dS = (dSr + dSl) / 2;
    d_theta = (dSr - dSl) / WHEEL_SEP; // change in the robots orientation
    
	//----------------------------------------//

	//----------initialize u_k A_k, B_k---------//

    u_k[0] = dSl ;
    u_k[1] = dSr ;
    u_k[2] = 0;

    A_k[0][0] = 1;
	A_k[0][1] = 0;
	A_k[0][2] = -dS*sin(x_hat[2]+d_theta/2);
	A_k[1][0] = 0;
	A_k[1][1] = 1;
	A_k[1][2] = dS*cos(x_hat[2]+d_theta/2);
	A_k[2][0] = 0;
	A_k[2][1] = 0;
	A_k[2][2] = 1;    

	/*printf("A_k:");
	print_mat(A_k);*/

	mat_trans(A_k,A_k_trans); 

	/*printf("A_k_trans:");
	print_mat(A_k_trans);*/


	B_k[0][0] = 1/2 * cos(x_hat[2] + d_theta/2) + dS / (2 * WHEEL_SEP) * sin(x_hat[2] + d_theta/2);
	B_k[0][1] = 1/2 * cos(x_hat[2] + d_theta/2) - dS / (2 * WHEEL_SEP) * sin(x_hat[2] + d_theta/2);
	B_k[0][2] = 0;
	B_k[1][0] = 1/2 * sin(x_hat[2] + d_theta/2) - dS / (2 * WHEEL_SEP) * cos(x_hat[2] + d_theta/2);
	B_k[1][1] = 1/2 * sin(x_hat[2] + d_theta/2) + dS / (2 * WHEEL_SEP) * cos(x_hat[2] + d_theta/2);
	B_k[1][2] = 0;
	B_k[2][0] = -1/WHEEL_SEP;
	B_k[2][1] = 1/WHEEL_SEP;
	B_k[2][2] = 0;

	/*printf("B_k:");
	print_mat(B_k);*/

	//------------Kalman algorithm--------------//
	//------------------------------------------//

	//----x_hat=A_k*x_hat+B_k*u_k-------------//



	mult_matrices_vect_3x3(A_k,x_hat,result_vect);
	copy_vect(result_vect,x_hat);
	mult_matrices_vect_3x3(B_k,u_k,result_vect);
	vect_add(x_hat,result_vect,x_hat,1);

	/*printf("4:");
	print_vect(x_hat);*/

	//-----P_k =A_k*P_k*A_K_trans+Q_k-----------//
	/*printf("4.1:");
	print_mat(P_hat);*/

	mult_matrices_3x3(A_k,P_hat,result_matrix);

	/*printf("4.2:");
	print_mat(result_matrix);*/

	mult_matrices_3x3(result_matrix,A_k_trans,P_hat);

	/*printf("4.3:");
	print_mat(P_hat);*/


	mat_add(P_hat,kalman_pos->Q,P_hat,1);
	/*printf("5:");
	print_mat(P_hat);*/

	//---------err_k = z_k-x_hat---------------//
	vect_add(z_k,x_hat,err_k,0);

	/*printf("6:");
	print_vect(err_k);*/

	//-------------S_k = P_hat + R_K----------//
	mat_add(P_hat,kalman_pos->R,S_k,1);

	/*printf("7:");
	print_mat(S_k);*/

	//-------------K_k = P_hat*S_k^-1---------//

	//set_mat_0(result_matrix);

	if (!inv_mat_3x3(S_k,result_matrix))
		{
			printf("singular\n");
			return;
		}

	mult_matrices_3x3(P_hat,result_matrix,K_k);

	/*printf("K_k:");
	print_mat(K_k);*/

	//---kalman_pos->x,y,theta = x_hat + K_k * err_k---//



	mult_matrices_vect_3x3(K_k,err_k,result_vect);
	vect_add(x_hat,result_vect,result_vect,1);
	kalman_pos->x = result_vect[0];
	kalman_pos->y = result_vect[1];
	kalman_pos->theta = result_vect[2];
 
 	//----kalman_pos->P_k = (I-K_k)P_hat----//


	mat_add(I,K_k,result_matrix,0);
	mult_matrices_3x3(result_matrix,P_hat,kalman_pos->P_k);


	//printf ( "%f %f %f;\n",kalman_pos->x,kalman_pos->y,kalman_pos->theta);
	
	// update time stamp for next loop
	kalman_pos->last_t = inputs->t;
}






NAMESPACE_CLOSE();