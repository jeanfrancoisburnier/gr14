#include "kalman_gr14.h"
#include "odometry_gr14.h"
#include "triangulation_gr14.h"
#include "useful_gr14.h"

NAMESPACE_INIT(ctrlGr14);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void kalman(CtrlStruct *cvs)
{
	// variable declaration
	RobotPosition *rob_pos;
	RobotPosition *triang_pos;
	RobotPosition *kalman_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	triang_pos = cvs->triang_pos;
	kalman_pos = cvs->kalman_pos;
}

NAMESPACE_CLOSE();
