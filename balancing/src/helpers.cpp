/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and filtering...
 */

#include "helpers.h"

using namespace Eigen;
using namespace std;

/* ******************************************************************************************** */
// Constants for the robot kinematics

const double wheelRadius = 10.5; 							///< Radius of krang wheels in inches
const double distanceBetweenWheels = 27.375; 	///< Distance Between krang's wheels in inches 

/* ******************************************************************************************** */
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < 1e-5) mat2(i,j) = 0.0;
	return mat2;
}

/* ******************************************************************************************** */
// Setup the indices for the motor groups

int left_arm_ids_a [7] = {11, 13, 15, 17, 19, 21, 23}; 
int right_arm_ids_a [7] = {12, 14, 16, 18, 20, 22, 24}; 
int imuWaist_ids_a [2] = {5, 8};
vector <int> left_arm_ids (left_arm_ids_a, left_arm_ids_a + 7);						
vector <int> right_arm_ids (right_arm_ids_a, right_arm_ids_a + 7);	
vector <int> imuWaist_ids (imuWaist_ids_a, imuWaist_ids_a + 2);		

/* ******************************************************************************************** */
/// Updates the dart robot representation
void updateDart (double imu) {

	// Update imu and waist values
	double waist_val = (waist.pos[0] - waist.pos[1]) / 2.0;
	Vector2d imuWaist_vals (-imu + M_PI_2, waist_val);
	robot->setConfig(imuWaist_ids, imuWaist_vals);

	// Update the robot state
	Vector7d larm_vals = eig7(llwa.pos), rarm_vals = eig7(rlwa.pos);
	robot->setConfig(left_arm_ids, larm_vals);
	robot->setConfig(right_arm_ids, rarm_vals);
}

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt) {

	// Read imu
	double imu, imuSpeed;
	getImu(&imuChan, imu, imuSpeed, dt, kf); 

	// Read Motors 
	somatic_motor_update(&daemon_cx, &amc);
	somatic_motor_update(&daemon_cx, &waist);
	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_update(&daemon_cx, &rlwa);

	// Update the dart robot representation and get the center of mass (decrease height of wheel)
	updateDart(imu);
	Vector3d com = robot->getWorldCOM();
	com(2) -= 0.264;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2));
	state(1) = imuSpeed;
	state(2) = (amc.pos[0] + amc.pos[1])/2.0 + imu;
	state(3) = (amc.vel[0] + amc.vel[1])/2.0 + imuSpeed;
	state(4) = wheelRadius * (amc.pos[0] - amc.pos[1]) / distanceBetweenWheels;
	state(5) = wheelRadius * (amc.vel[0] - amc.vel[1]) / distanceBetweenWheels;
}

/* ******************************************************************************************** */
/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState) {

	// First, set the balancing angle and velocity to zeroes
	refState(0) = refState(1) = 0.0;

	// Set the distance and heading velocities using the joystick input
	static const double kMaxForwVel = 2.0, kMaxSpinVel = 3.0;
	refState(3) = kMaxForwVel * js_forw;
	refState(5) = kMaxSpinVel * js_spin;

	// Integrate the reference positions with the current reference velocities
	refState(2) += dt * refState(3);
	refState(4) += dt * refState(5);
}

/* ******************************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick 
bool getJoystickInput(double& js_forw, double& js_spin) {

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return false;

	// Change the gains with the given joystick input
	double deltaTH = 0.2;					// deltaX = 0.02;
	int64_t* b = &(js_msg->buttons->data[0]);
	for(size_t i = 0; i < 4; i++)
		if(b[i] == 1)
			K_bal(i % 2) += ((i < 2) ? deltaTH : -deltaTH);
	
	// Ignore the joystick statements for the arm control 
	if((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1)) {
		js_forw = js_spin = 0.0;
		return true;
	}

	// Set the values for the axis
	double* x = &(js_msg->axes->data[0]);
	js_forw = -x[1];
	js_spin = 0.0; 
	return true;
}

/* ********************************************************************************************* */
/// Sets a global variable ('start') true if the user presses 's'
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='s') break;
	}
	start = true;
}


/* ********************************************************************************************* */
/// Computes the imu value from the imu readings
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
		filter_kalman_t* kf) {

	// ======================================================================
	// Get the readings

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	_imu = atan2(newX, imu_msg->data[2]); 
	_imuSpeed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// ======================================================================
	// Filter the readings

	// Skip if a filter is not provided
	if(kf == NULL) return;

	// Setup the data
	kf->z[0] = _imu, kf->z[1] = _imuSpeed;

	// Setup the time-dependent process matrix
	kf->A[0] = kf->A[3] = 1.0;
	kf->A[2] = dt;

	// Setup the process noise matrix
	static const double k1 = 2.0;
	static const double k1b = 5.0;
	kf->R[0] = (dt*dt*dt*dt) * k1 * (1.0 / 4.0);
	kf->R[1] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[2] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[3] = (dt*dt) * k1b;
	
	// First make a prediction of what the reading should have been, then correct it
	filter_kalman_predict(kf);
	filter_kalman_correct(kf);

	// Set the values
	_imu = kf->x[0], _imuSpeed = kf->x[1];
}
