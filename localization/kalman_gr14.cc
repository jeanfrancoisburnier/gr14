/*
 *We use the kalman algorithm found on wikipedia https://en.wikipedia.org/wiki/Kalman_filter
 * However in this case we taqke the measurment to be the values of x y and theta being the values
 * computed with the triangulation algorithm which allows us to use a H matrix which is identity simplifying 
 * greatly the algebra.
 * The matrix A and B are the same as the one used in the paper "Three-state Extended Kalman Filter for Mobile
 * Robot Localization" from Evgeni Kiriy Martin Buehler in CMU. The different computation for the Kalman process are 
 * detailed afterwards.
*/




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

/*! \brief Kalman
 * 
 * \param[in,out] cvs controller main structure
 */
void kalman(CtrlStruct *cvs)
{
	// variable declaration
	RobotPosition *triang_pos;
	KalmanStruct *kalman_pos;
	CtrlIn *inputs;

	inputs  = cvs->inputs;
	triang_pos = cvs->triang_pos;
	kalman_pos = cvs->kalman_pos;

	/* Description of the variables used
	 *
	 * z_k position computed with the triangulation method
	 * x_hat Initialized with the previous kalman pos and it will be updated during the kalman call
	 * P_hat The covariance matrix initialized with the previous matrix, will be updated during the algorithm
	 * A_k, A_k_trans, B_k matrix used in the kalman algorithm see wikipedia page for details
	 * S_k intermiditate matrix used in the Kalman algorithm see wikipedia
	 * K_k kalman gain 
	 * u_k inputs for the a priori update here it is dSl,dSr see below
	 * I 3 by 3 identity matrix
	 * err_k difference between the calculated position and the measured position 
	 * dSl,dSr,dS,d_theta respectively the computed displacement of the left wheel, right wheel center of robot
	 * and change in the robot's orientation since the last call of the function
	 * dt change in time since the last computation of Kalman
	 * r_sp, l_sp r wheel and left wheel speed
	 * result_matrix, result_vect result vector and matrix used as temporary in the algorithm
	 *
	*/	
	double z_k[3] = {(*triang_pos).x,(*triang_pos).y,(*triang_pos).theta};
	double x_hat[3] = {(*kalman_pos).x,(*kalman_pos).y,(*kalman_pos).theta};
	double P_hat[3][3];
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
	double result_matrix[3][3];
	double result_vect[3];

	//condition if the triangulation pos and the previous pos are too different it will ignore 
	//particularly useful in the beggining before triagulation is working properly
	if ((std::abs(x_hat[0]-z_k[0])>0.1 || std::abs(x_hat[1]-z_k[1])>0.1 || std::abs(x_hat[2]-z_k[2])>0.1) && inputs->t<-14.6)
		{
			return;
		}

	//------compute dSl, dSr, d_theta and dS for the matrices A and B--------//
	//-----the method used is the same as the one used in odometry-----------//
	r_sp = wheel_speed_meter(inputs->r_wheel_speed,WHEEL_RAD); 
	l_sp = wheel_speed_meter(inputs->l_wheel_speed,WHEEL_RAD);

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
    d_theta = (dSr - dSl) / WHEEL_SEP;
    
	//----------------------------------------//

	//----------initialize u_k A_k, B_k---------//
	// Formulas are found in the paper mentionned above

    A_k[0][0] = 1;
	A_k[0][1] = 0;
	A_k[0][2] = -dS*sin(x_hat[2]+d_theta/2);
	A_k[1][0] = 0;
	A_k[1][1] = 1;
	A_k[1][2] = dS*cos(x_hat[2]+d_theta/2);
	A_k[2][0] = 0;
	A_k[2][1] = 0;
	A_k[2][2] = 1;    

	mat_trans(A_k,A_k_trans); 

    /*
     *
     * The computation are done all in 3 by 3 matrix (less sub function and easier algebra)
     * since u_k and B_k are supposed to be a R_2 vector and a 3 by 2 matrix I have simply added zeros
     * to make the R_3 and 3 by 3
    */ 
    u_k[0] = dSl ;
    u_k[1] = dSr ;
    u_k[2] = 0; 

	B_k[0][0] = 1/2 * cos(x_hat[2] + d_theta/2) + dS / (2 * WHEEL_SEP) * sin(x_hat[2] + d_theta/2);
	B_k[0][1] = 1/2 * cos(x_hat[2] + d_theta/2) - dS / (2 * WHEEL_SEP) * sin(x_hat[2] + d_theta/2);
	B_k[0][2] = 0;
	B_k[1][0] = 1/2 * sin(x_hat[2] + d_theta/2) - dS / (2 * WHEEL_SEP) * cos(x_hat[2] + d_theta/2);
	B_k[1][1] = 1/2 * sin(x_hat[2] + d_theta/2) + dS / (2 * WHEEL_SEP) * cos(x_hat[2] + d_theta/2);
	B_k[1][2] = 0;
	B_k[2][0] = -1/WHEEL_SEP;
	B_k[2][1] = 1/WHEEL_SEP;
	B_k[2][2] = 0;



	//------------Kalman algorithm--------------//
	//--Details found in the wikipedia page mentionned at the top--//

	//----x_hat=A_k*x_hat+B_k*u_k---------------//

	mult_matrices_vect_3x3(A_k,x_hat,result_vect);
	copy_vect(result_vect,x_hat);
	mult_matrices_vect_3x3(B_k,u_k,result_vect);
	vect_add(x_hat,result_vect,x_hat,1);

	//-----P_k =A_k*P_k*A_K_trans+Q_k----------//

	mult_matrices_3x3(A_k,P_hat,result_matrix);
	mult_matrices_3x3(result_matrix,A_k_trans,P_hat);
	mat_add(P_hat,kalman_pos->Q,P_hat,1);

	//---------err_k = z_k-x_hat---------------//

	vect_add(z_k,x_hat,err_k,0);

	//-------------S_k = P_hat + R_K-----------//

	mat_add(P_hat,kalman_pos->R,S_k,1);

	//-------------K_k = P_hat*S_k^-1----------//

	if (!inv_mat_3x3(S_k,result_matrix))
		{
			printf("singular\n");
			return;
		}

	mult_matrices_3x3(P_hat,result_matrix,K_k);

	//---kalman_pos->x,y,theta = x_hat + K_k * err_k---//

	mult_matrices_vect_3x3(K_k,err_k,result_vect);
	vect_add(x_hat,result_vect,result_vect,1);
	kalman_pos->x = result_vect[0];
	kalman_pos->y = result_vect[1];
	kalman_pos->theta = result_vect[2];
 
 	//------kalman_pos->P_k = (I-K_k)P_hat-----//

	mat_add(I,K_k,result_matrix,0);
	mult_matrices_3x3(result_matrix,P_hat,kalman_pos->P_k);

	//--------end of the Kalman filter---------//

	//printf ( "%f %f %f;\n",kalman_pos->x,kalman_pos->y,kalman_pos->theta);
	set_output(cvs->kalman_pos->x,"x");
	set_output(cvs->kalman_pos->y,"y");
	
	// update time stamp for next loop
	kalman_pos->last_t = inputs->t;
}

NAMESPACE_CLOSE();
