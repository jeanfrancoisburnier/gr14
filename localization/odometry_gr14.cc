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

	RobotPosition *rob_pos;
	CtrlIn *inputs;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

    r_sp = wheel_speed_meter(inputs->r_wheel_speed,wheel_rad); // right wheel speed
	l_sp = wheel_speed_meter(inputs->l_wheel_speed,wheel_rad); // left wheel speed

	// time
	dt = inputs->t - rob_pos->last_t; // time increment since last call

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
    
    dx = dS * cos(rob_pos->theta + d_theta/2);
    dy = dS * sin(rob_pos->theta + d_theta/2);

	// ----- odometry computation end ----- //

	// last update time
	rob_pos->last_t = inputs->t;
    rob_pos->x = rob_pos->x + dx;
    rob_pos->y = rob_pos->y + dy;
    rob_pos->theta = limit_angle(rob_pos->theta + d_theta);
}

double wheel_speed_meter(double wheel_speed_rad,double wheel_radius) //speed form radians per second to meter per second
{
    double wheel_speed_m = wheel_speed_rad*wheel_radius;
    return wheel_speed_m;
}

NAMESPACE_CLOSE();
