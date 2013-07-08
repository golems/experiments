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
ach_channel_t imuChan;
ach_channel_t waistChan;			///< the state channel for the waist module
ach_channel_t leftArmChan;		///< the state channel for the left arm modules
ach_channel_t rightArmChan;		///< the state channel for the right arm modules

/* ********************************************************************************************* */
/// Gets the data from the channels
void getData (Eigen::VectorXd& q, Eigen::VectorXd& dq, double dt, filter_kalman_t* kf) {

	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);

	// Get the data from the motors
	int r;
	Somatic__MotorState *waist, *leftArm, *rightArm;
	assert(((waist = getMotorMessage(waistChan)) != NULL) && "Waist call failed");
	assert(((leftArm = getMotorMessage(leftArmChan)) != NULL) && "leftArm call failed");
	assert(((rightArm = getMotorMessage(rightArmChan)) != NULL) && "rightArm call failed");
	
	// Get the data from imu
	double imu, imuSpeed;
	getImu(&imuChan, imu, imuSpeed, dt, kf);

	// Store data in the output structure
	// FIXME: This imu value is not compatible to dart unless changed to -imu+M_PI/2
	// the current value is compatible to the dynamics.cpp. We need to make dynamics.cpp
	// compatible to dart
	q(5) = imu; dq(5) = imuSpeed;
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
		getData(q, dq, dt, kf);

		// Compute the center of mass with the sensor data

	}

	// Once the main loop returns, the program is done. Send the stopping event message.
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
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
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	somatic_d_channel_open(&daemon_cx, &leftArmChan, "llwa-state", NULL);
	somatic_d_channel_open(&daemon_cx, &rightArmChan, "rlwa-state", NULL);

	// Initialize kalman filter for the imu and set the measurement and meas. noise matrices
	kf = new filter_kalman_t;
	filter_kalman_init(kf, 2, 0, 2);
	kf->C[0] = kf->C[3] = 1.0;
	kf->Q[0] = kf->Q[3] = 1e-3;
}

/* ********************************************************************************************* */
/// Destroy the channel and daemon resources
void destroy () {
	somatic_d_channel_close(&daemon_cx, &imuChan);
	somatic_d_channel_close(&daemon_cx, &waistChan);	 
	somatic_d_channel_close(&daemon_cx, &leftArmChan); 
	somatic_d_channel_close(&daemon_cx, &rightArmChan);
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
