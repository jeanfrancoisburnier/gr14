#include "odometry_gr14.h"
#include "useful_gr14.h"
#include "init_pos_gr14.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr14);

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	double r_sp, l_sp;
	double dt;
    double wheel_sep = 0.225; //distance between both wheels
    double wheel_rad = 0.030; //radiius of a wheel
    double dS=0; //distance traveled by robot
    double dSl=0;//distance traveled by right and left wheel
    double dSr;
    double d_theta;
    double dx;
    double dy;

	KalmanStruct *kalman_pos;
	CtrlIn *inputs;

	// variables initialization
	inputs  = cvs->inputs;
	kalman_pos = cvs->kalman_pos;

    r_sp = wheel_speed_meter(inputs->r_wheel_speed,wheel_rad); // right wheel speed
	l_sp = wheel_speed_meter(inputs->l_wheel_speed,wheel_rad); // left wheel speed

	// time
	dt = inputs->t - kalman_pos->last_t; // time increment since last call

	// safety
	if (dt <= 0.0)
	{
		return;
	}

	// ----- odometry computation start ----- //

    dSl = dt * l_sp ;
    dSr = dt * r_sp ;
    
    dS = (dSr + dSl) / 2;
    d_theta = (dSr - dSl) / wheel_sep; // change in the robots orientation
    
    dx = dS * cos(kalman_pos->theta + d_theta/2);
    dy = dS * sin(kalman_pos->theta + d_theta/2);

	// ----- odometry computation end ----- //

	// last update time
	kalman_pos->last_t = inputs->t;
    kalman_pos->x = kalman_pos->x + dx;
    kalman_pos->y = kalman_pos->y + dy;
    kalman_pos->theta = limit_angle(kalman_pos->theta + d_theta);
}



NAMESPACE_CLOSE();
