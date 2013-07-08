/**
 * @file 01-com.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This executable shows how to estimate the center of mass of the Krang.
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include "helpers.h"

#define getMotorMessage(x) (SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, \
	&protobuf_c_system_allocator, 1024, &x, &abstime))

/* ******************************************************************************************** */
// Communication and filtering

filter_kalman_t *kf;						///< The kalman filter to smooth the imu readings
somatic_d_t daemon_cx;
ach_channel_t amcChan;
ach_channel_t imuChan;
ach_channel_t waistChan;			///< the state channel for the waist module
ach_channel_t leftArmChan;		///< the state channel for the left arm modules
ach_channel_t rightArmChan;		///< the state channel for the right arm modules

double wheelInitial[2];					///< stores the initial state of the wheels

/* ********************************************************************************************* */
/// Gets the data from the channels
void getData (double dt, Eigen::VectorXd& q, Eigen::VectorXd& dq,	filter_kalman_t* kf) {

	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);

	// Get the data from the motors
	int r;
	Somatic__MotorState *waist, *leftArm, *rightArm, *amc;
	assert(((waist = getMotorMessage(waistChan)) != NULL) && "Waist call failed");
	assert(((amc = getMotorMessage(amcChan)) != NULL) && "Wheels call failed");
	assert(((leftArm = getMotorMessage(leftArmChan)) != NULL) && "leftArm call failed");
	assert(((rightArm = getMotorMessage(rightArmChan)) != NULL) && "rightArm call failed");
	
	// Get the data from imu
	double imu, imuSpeed;
	getImu (imu, imuSpeed);

	// Compensate for imu from wheel encoders and subtract intial wheel position
	amc->position->data[0] += imu-wheelInitial[0];
	amc->position->data[1] += imu-wheelInitial[1];

	// Store data in the output structure
	// FIXME: This imu value is not compatible to dart unless changed to -imu+M_PI/2
	// the current value is compatible to the dynamics.cpp. We need to make dynamics.cpp
	// compatible to dart
	q(5) = imu; dq(5) = imuSpeed;
	q(6) = amc->position->data[0]; dq(6) = amc->velocity->data[0];
	q(7) = amc->position->data[1]; dq(7) = amc->velocity->data[1];
	q(8) = (waist->position->data[0] - waist->position->data[1])/2.0;
	dq(8) = (waist->velocity->data[0] - waist->velocity->data[1])/2.0;
	for (int i = 0; i < 7; i++) {
		q(10 + 2*i) = leftArm->position->data[i]; dq(10 + 2*i) = leftArm->velocity->data[i];
		q(11 + 2*i) = rightArm->position->data[i]; dq(11 + 2*i) = rightArm->velocity->data[i];
	}
	
	// Filter the data
	filterState(dt, kf, q, dq);
}

/* ******************************************************************************************** */
void run() {

	// Send the event massage
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing the joint values and computing the com until sigterm received
	struct timespec t_now, t_prev;
	t_prev = aa_tm_now();
	Eigen::VectorXd q (24), dq (24), leftFt(6), rightFt(6);
	while (!somatic_sig_received ) {

		// Get the current time and compute the difference
		t_now = aa_tm_now();
		double dt = (double) aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));

		// Get the data from ach channels and filter
		getData(dt, q, dq, kf);
	}

	// Once the main loop returns, the program is done. Send the stopping event message.
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}


/* ********************************************************************************************* */
/// Gets initial wheel positions to be subtracted from all subsequent readings. This is because
/// the initial values are to be treated as zeros.
void getWheelInitial(double* _wheelInitial[]) {
	
	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	
	// Reads amc position from the amc state channel
	Somatic__MotorState* amc; int r;
	assert(((amc = getMotorMessage(amcChan)) != NULL) && "Wheels call failed");

	// Get imu position
	double imu, imuSpeed;
	getImu(imu, imuSpeed);

	// Store in amcOffset the sum of amc-encoders and imu value. This is done to compensate for the
	// effect of imu rotation in encoder readings.
	*_wheelInitial[0] = amc->position->data[0]+imu;
	*_wheelInitial[1] = amc->position->data[1]+imu;
}

/* ********************************************************************************************* */
/// Initializes the ach channels needed to retrieve the state of the robot
void init () {

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-com";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the channels to the sensors
	somatic_d_channel_open(&daemon_cx, &amcChan, "amc-state", NULL);
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	somatic_d_channel_open(&daemon_cx, &leftArmChan, "llwa-state", NULL);
	somatic_d_channel_open(&daemon_cx, &rightArmChan, "rlwa-state", NULL);

	// Get first reading from amc, call it amc_offset, to be subtracted from all subsequent readings
	getWheelInitial(&wheelInitial);

	// Initialize kalman filter
	kf = new filter_kalman_t;
	filter_kalman_init(kf, 8, 0, 8);
}

/* ********************************************************************************************* */
/// Destroy the channel and daemon resources
void destroy () {
	somatic_d_channel_close(&daemon_cx, &imuChan);
	somatic_d_channel_close(&daemon_cx, &waistChan);	 
	somatic_d_channel_close(&daemon_cx, &leftArmChan); 
	somatic_d_channel_close(&daemon_cx, &rightArmChan);
	somatic_d_channel_close(&daemon_cx, &ftLeftChan);
	somatic_d_channel_close(&daemon_cx, &ftRightChan);
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}
