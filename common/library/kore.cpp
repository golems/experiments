/**
 * @file kore.cpp
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The main source file for the Krang support file which has the definition for the 
 * Hardware struct constructor.
 */

#include "kore.h"

using namespace Eigen;

namespace Krang {
	
/* ******************************************************************************************** */
Hardware::Hardware (Mode mode_, somatic_d_t* daemon_cx_, dynamics::SkeletonDynamics* robot_) {

	// Set the local variables
	daemon_cx = daemon_cx_;
	robot = robot_;
	mode = mode_;

	// Initialize all the 'optional' pointers to nulls and sanity check the inputs
	lft = rft = NULL;
	amc = larm = rarm = torso = grippers = NULL;
	waistCmdChan = NULL;
	assert((daemon_cx != NULL) && "Can not give null daemon context to hardware constructor");
	assert((robot != NULL) && "Can not give null dart kinematics to hardware constructor");
	
	// Initialize the imu sensor and average the first 500 values
	initImu();

	// Define the pos/vel limits for the motor groups
	VectorXd lim7 = VectorXd::Ones(7) * 1024.1, lim2 = VectorXd::Ones(2) * 1024.1; 
	VectorXd lim1 = VectorXd::Ones(1) * 1024.1;

	// Initialize the Schunk (+ Robotiq) motor groups
	if(mode & MODE_LARM) 
		initMotorGroup(larm, "llwa-cmd", "llwa-state", -lim7, lim7, -lim7, lim7);	
	if(mode & MODE_RARM) 
		initMotorGroup(rarm, "rlwa-cmd", "rlwa-state", -lim7, lim7, -lim7, lim7);	
	if(mode & MODE_TORSO) 
		initMotorGroup(torso, "torso-cmd", "torso-state", -lim1, lim1, -lim1, lim1);	
	if(mode & MODE_GRIPPERS) 
		initMotorGroup(torso, "grippers-cmd", "grippers-state", -lim2, lim2, -lim2, lim2);	

	// Initialize the wheel motor groups which depend on imu readings to get absolute wheel positions
	if(mode & MODE_AMC) initWheels();

	// Initialize the ach channels to the waist daemon and the waist state channel. The daemon moves
	// the motors in unison with current control and pciod updates their state on the state channel.
	if(mode & MODE_WAIST) {
		somatic_d_channel_open(daemon_cx, waistCmdChan, "waistd-cmd", NULL);
		somatic_motor_init(daemon_cx, waist, 2, NULL, "waist-state");
		somatic_motor_update(daemon_cx, waist);
	}

	// Updates the robot kinematics
	updateKinematics();
	
	// After initializing the rest of the robot (need kinematics), we can initialize f/t sensors. 
	// TODO: Determine the type of the gripper from the mode
	if(mode & MODE_LARM) lft = new FT(FT::GRIPPER_TYPE_ROBOTIQ, daemon_cx, robot, LEFT);
	if(mode & MODE_RARM) rft = new FT(FT::GRIPPER_TYPE_ROBOTIQ, daemon_cx, robot, RIGHT);
}

/* ******************************************************************************************** */
void Hardware::updateSensors (double dt) {

	// Update the lower body motors to get the current values
	if(mode & MODE_AMC) somatic_motor_update(daemon_cx, amc);
	if(mode & MODE_TORSO) somatic_motor_update(daemon_cx, torso);
	if(mode & MODE_WAIST) somatic_motor_update(daemon_cx, waist);

	// Update the imu
	getImu(imu_chan, imu, imuSpeed, dt, kfImu); 

	// Update the arms and ft sensors if required
	if(mode & MODE_LARM) {
		somatic_motor_update(daemon_cx, larm);
		lft->updateExternal();
	}
	if(mode & MODE_RARM) {
		somatic_motor_update(daemon_cx, rarm);
		rft->updateExternal();
	}
}

/* ******************************************************************************************** */
void Hardware::updateKinematics () {

	// Update imu and waist values
	assert((mode & MODE_WAIST) && "This code assumes that the robot at least has the waist modules");
	double waist_val = (waist->pos[0] - waist->pos[1]) / 2.0;
	Vector2d imuWaist_vals (-imu + M_PI_2, waist_val);
	robot->setConfig(imuWaist_ids, imuWaist_vals);

	// Update the arms state
	if(mode & MODE_LARM) {
		Vector7d larm_vals = eig7(larm->pos);
		robot->setConfig(left_arm_ids, larm_vals);
	}
	if(mode & MODE_RARM) {
		Vector7d rarm_vals = eig7(rarm->pos);
		robot->setConfig(right_arm_ids, rarm_vals);
	}
}

/* ******************************************************************************************** */
void Hardware::initImu () {

	// Initialize the ach channel
	somatic_d_channel_open(daemon_cx, imu_chan, "imu-data", NULL);

	// Average the first 500 readings
	imu = imuSpeed = 0.0;
	for(int i = 0; i < 500; i++) {
		double tempImu, tempImuSpeed;
		getImu(imu_chan, tempImu, tempImuSpeed, 0.0, NULL); 
		imu += tempImu, imuSpeed += tempImuSpeed;
	}
	imu /= 500.0, imuSpeed /= 500.0;

	// Create the kalman filter
	kfImu = new filter_kalman_t;
	filter_kalman_init(kfImu, 2, 0, 2);
	kfImu->C[0] = kfImu->C[3] = 1.0;
	kfImu->Q[0] = kfImu->Q[3] = 1e-3;
	kfImu->x[0] = imu, kfImu->x[1] = imuSpeed;
}

/* ******************************************************************************************** */
void Hardware::initWheels () {

	// Initialize the motor group (do we need to set any limits?)
	somatic_motor_init(daemon_cx, amc, 2, "amc-cmd", "amc-state");

	// Update and reset them
	somatic_motor_cmd(daemon_cx, amc, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 2, NULL);
	usleep(1e5);
	somatic_motor_update(daemon_cx, amc);
	usleep(1e5);

	// Set the offset values to amc motor group so initial wheel pos readings are zero
	double pos_offset[2] = {-amc->pos[0] - imu, -amc->pos[1] - imu};
	aa_fcpy(amc->pos_offset, pos_offset, 2);
	usleep(1e5);
}

/* ******************************************************************************************** */
void Hardware::initMotorGroup (somatic_motor_t*& motors, const char* cmd_name, const char* 
		state_name, VectorXd minPos, VectorXd maxPos, VectorXd minVel, VectorXd maxVel) {

	// Initialize the somatic motor struct with the channel names and the number of modules
	size_t numModules = minPos.size();
	somatic_motor_init(daemon_cx, motors, numModules, cmd_name, state_name);

	// Set the min/max values for the pos/vel fields' valid and limit values
	double** limits [] = {
		&motors->pos_valid_min, &motors->pos_limit_min, &motors->pos_valid_max, &motors->pos_limit_max, 
		&motors->vel_valid_min, &motors->vel_limit_min, &motors->vel_valid_max, &motors->vel_limit_max};
	VectorXd* inputs [] = {&minPos, &maxPos, &minVel, &maxVel};
	for(size_t i = 0; i < 8; i++) aa_fcpy(*limits[i], inputs[i/2]->data(), numModules);

	// Update and reset them
	somatic_motor_cmd(daemon_cx, motors, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, numModules, NULL);
	usleep(1e5);
	somatic_motor_update(daemon_cx, motors);
	usleep(1e5);
}

};
