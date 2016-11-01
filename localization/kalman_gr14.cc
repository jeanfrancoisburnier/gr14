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

	double x_triang,y_triang,theta_triang;
	double x_hat[3] = {kalman_pos->x,kalman_pos->y,kalman_pos->theta};
	double P_hat[3][3];
	double A_k[3][3];
	double A_k_trans[3][3] = { 0 };
	double B_k[3][3];
	double S_k[3][3];
	double K_k[3][3];
	double u_k[3];
	double I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	double err_k;
	double dSl,dSr,dS,d_theta;
	double dt;
	double r_sp,l_sp;

	// variables initialization

	//rob_pos = cvs->rob_pos;
	x_triang = triang_pos->x;
	y_triang = triang_pos->y;
	theta_triang = triang_pos->theta;
	err_k = 0;


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

	mat_trans(A_k,A_k_trans); 

	B_k[0][0] = 1/2 * cos(x_hat[2] + d_theta/2) + dS / (2 * WHEEL_SEP) * sin(x_hat[2] + d_theta/2);
	B_k[0][1] = 1/2 * cos(x_hat[2] + d_theta/2) - dS / (2 * WHEEL_SEP) * sin(x_hat[2] + d_theta/2);
	B_k[0][2] = 0;
	B_k[1][0] = 1/2 * sin(x_hat[2] + d_theta/2) - dS / (2 * WHEEL_SEP) * cos(x_hat[2] + d_theta/2);
	B_k[1][1] = 1/2 * sin(x_hat[2] + d_theta/2) + dS / (2 * WHEEL_SEP) * cos(x_hat[2] + d_theta/2);
	B_k[1][2] = 0;
	B_k[2][0] = -1/WHEEL_SEP;
	B_k[2][1] = 1/WHEEL_SEP;
	B_k[2][2] = 0;


	// update time stamp for next loop
	kalman_pos->last_t = inputs->t;
}






NAMESPACE_CLOSE();