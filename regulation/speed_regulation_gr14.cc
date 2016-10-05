#include "speed_regulation_gr14.h"
#include "useful_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/*! \brief wheel speed regulation
 * 
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	double r_sp, l_sp;
	double dt;

	// variables declaration
	CtrlIn *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg  = cvs->sp_reg;

	// wheel speeds
	r_sp = inputs->r_wheel_speed;
	l_sp = inputs->l_wheel_speed;

	// time
	dt = inputs->t - sp_reg->last_t; // time interval since last call

	// ----- Wheels regulation computation start ----- //

	double TI = 0.5;
	double KP = 50.0;

	sp_reg->int_error_r = sp_reg->int_error_r + r_sp_ref-r_sp;
	sp_reg->int_error_l = sp_reg->int_error_l + l_sp_ref-l_sp;

	// wheel commands
	outputs->wheel_commands[R_ID] = KP*(r_sp_ref-r_sp + sp_reg->int_error_r*dt/TI);
	outputs->wheel_commands[L_ID] = KP*(l_sp_ref-l_sp + sp_reg->int_error_l*dt/TI);

	outputs->wheel_commands[R_ID] = limit_range(outputs->wheel_commands[R_ID], -100, 100);
	outputs->wheel_commands[L_ID] = limit_range(outputs->wheel_commands[L_ID], -100, 100);
	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
}

NAMESPACE_CLOSE();
