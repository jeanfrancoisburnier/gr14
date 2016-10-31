#include "kalman_gr14.h"
#include "odometry_gr14.h"
#include "triangulation_gr14.h"
#include "useful_gr14.h"


#define wheel_sep 0.225
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
	//RobotPosition *triang_pos;
	KalmanStruct *kalman_pos;

	// variables initialization
	//rob_pos = cvs->rob_pos;
	//triang_pos = cvs->triang_pos;
	//kalman_pos = cvs->kalman_pos;

	double x_hat , y_hat , theta_hat;
	double P_hat[3][3] = { 0 };
	double A_k[3][3] = { 0 };
	double B_k[3][2] = { 0 };
	double S_k[3][3] = { 0 };
	double K_k[3][3] = { 0 };
	double I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	double err_k = 0;
	double delta_Dkl,delta_Dkr,delta_Dk,delta_phi;

}

NAMESPACE_CLOSE();