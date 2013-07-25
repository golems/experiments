/**
 * @file cortex.cpp
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The main source file for the Krang support file which has the definition for the 
 * Hardware struct constructor.
 */

#include "cortex.h"

using namespace Eigen;

/* ******************************************************************************************** */
Hardware::Hardware (Mode mode, somatic_d_t* daemon_cx) {

	// Initialize all the 'optional' pointers to nulls
	lft = rft = NULL;
	amc = larm = rarm = torso = grippers = NULL;
	waistCmdChan = waistState = NULL;
	
	// Initialize the sensors - if we have arms, initialize the f/t sensors
	initImu();
	if(mode & MODE_LARM) initFT(LEFT);
	if(mode & MODE_RARM) initFT(RIGHT);

	// Define the pos/vel limits for the motor groups
	VectorXd lim7 = VectorXd::Ones(7) * 1024.1, lim2 = VectorXd::Ones(7) * 1024.1;

	// Initialize the Schunk (+ Robotiq) motor groups
	if(mode & MODE_LARM) 
		initMotorGroup(daemon_cx, larm, "llwa-cmd", "llwa-state", -lim7, lim7, -lim7, lim7);	
	if(mode & MODE_RARM) 
		initMotorGroup(daemon_cx, rarm, "rlwa-cmd", "rlwa-state", -lim7, lim7, -lim7, lim7);	
	if(mode & MODE_TORSO) 
		initMotorGroup(daemon_cx, torso, "torso-cmd", "torso-state", -100.0, 100.0, -100.0, 100.0);	
	if(mode & MODE_GRIPPERS) 
		initMotorGroup(daemon_cx, torso, "grippers-cmd", "grippers-state", -lim2, lim2, -lim2, lim2);	

	// Initialize the wheel motor groups which depend on imu readings to get absolute wheel positions
	if(mode & MODE_AMC) initWheels();

	// Initialize the interface to waist motors that is a daemon which moves them in unison with 
	// current control
	if(mode & MODE_WAIST) initWaist();
}

/* ******************************************************************************************** */

/* ******************************************************************************************** */
void initMotorGroup (somatic_d_t* daemon_cx, somatic_motor_t*& motors, const char* cmd_name,
		const char* state_name, VectorXd minPos, VectorXd maxPos, VectorXd minVel, VectorXd maxVel) {

	// Initialize the somatic motor struct with the channel names and the number of modules
	size_t numModules = minPos.size();
	somatic_motor_init(daemon_cx, motors, numModules, cmd_name, state_name);

	// Set the min/max values for the pos/vel fields' valid and limit values
	double** limits [] = {
		&motors.pos_valid_min, &motors.pos_limit_min, &motors.pos_valid_max, &motors.pos_limit_max, 
		&motors.vel_valid_min, &motors.vel_limit_min, &motors.vel_valid_max, &motors.vel_limit_max};
	VectorXd* inputs [] = {&minPos, &maxPos, &minVel, &maxVel};
	for(size_t i = 0; i < 8; i++) aa_fcpy(*limits[i], inputs[i/2]->data(), numModules);

	// Update and reset them
	somatic_motor_update(daemon_cx, motors);
	somatic_motor_cmd(daemon_cx, motors, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, numModules, NULL);
}
