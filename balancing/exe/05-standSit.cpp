/**
 * @file 05-standSit.cpp
 * @author Can Erdogan
 * @date July 14, 2013
 * @brief This demonstration shows the stand up and sit down process of Krang.
 * NOTE We are using a Vector6d to represent the state which has: th, th. x, x., psi, psi.
 * where . is the derivative, th is the imu angle, x is the distance along the heading and
 * psi is the heading angle.
 * NOTE We are not using psi in this angle but for generality, we will keep it.
 */

#include <iostream>
#include <pthread.h>
#include <unistd.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <filter.h>
#include <ach.h>

#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <Eigen/Dense>

#include "helpers.h"

using namespace Eigen;
using namespace std;
using namespace dynamics;

/* ******************************************************************************************** */
/// The continuous control loop which has 4 state variables, {x, x., psi, psi.}, where
/// x is for the position along the heading direction and psi is the heading angle. We create
/// reference x and psi values from the joystick and follow them with pd control.
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Initially the reference position and velocities are zero (don't move!)
	Vector6d refState, state;
	refState << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	// Unless an interrupt or terminate message is received, process the new message
	cout << "start..." << endl;
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	while(!somatic_sig_received) {

		bool debug = (c_++ % 20 == 0);

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		if(debug) cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << endl;
		// Get the current state and ask the user if they want to start
		getState(state, dt);
		if(debug) printf("balancing angle (deg): %lf\n", state(0) / M_PI * 180.0);
		
		// Get the joystick input for the js_forw and js_spin axes
		double js_forw = 0.0, js_spin = 0.0;
		bool gotInput = false;
		while(!gotInput) gotInput = getJoystickInput(js_forw, js_spin);

		// Determine the reference values for x and psi
		updateReference(js_forw, js_spin, dt, refState);
		if(debug) cout << "refState: " << refState.transpose() << endl;
		if(debug) cout << "state: " << state.transpose() << endl;
		
		// Choose the balancing or standing up control based on the imu angle
		Vector6d K;
		if(state(0) < -1.3) {
			K = K_stand;
			if(debug) cout << "Standing up mode: " << K.transpose() << endl;
		}
		else {
			K = K_bal;
			if(debug) cout << "Balancing mode: " << K.transpose() << endl;
		}

		// Compute the error term between reference and current, and weight with gains (no spin)
		Vector6d error = state - refState;
		double u = K.topLeftCorner<4,1>().dot(error.topLeftCorner<4,1>());
		double input [2] = {u, u};
		if(debug) printf("u: %lf\n", u);

		// Set the motor velocities
		if(start) 
			somatic_motor_cmd(&daemon_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Initialize the motor and daemons
void init() {

	// =======================================================================
	// Initialize the daemon, imu/joystick channels and kalman filter

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "03-wheels";
	somatic_d_init(&daemon_cx, &dopt);

	// Open the channel for imu
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	
	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// =======================================================================
	// Initialize the motors

	// Initialize the motor groupis and imu
	somatic_motor_init(&daemon_cx, &amc, 2, "amc-cmd", "amc-state");
	somatic_motor_init(&daemon_cx, &waist, 2, "waist-cmd", "waist-state");
	somatic_motor_init(&daemon_cx, &llwa, 7, "llwa-cmd", "llwa-state");
	somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
	
	// Set the min and maximum position and velocity valid/limit values for motors
	double ** limits[] = { 
		&amc.pos_valid_min, &amc.vel_valid_min, &amc.pos_limit_min, &amc.vel_limit_min, 
		&waist.pos_valid_min, &waist.vel_valid_min, &waist.pos_limit_min, &waist.vel_limit_min, 
		&llwa.pos_valid_min, &llwa.vel_valid_min, &llwa.pos_limit_min, &llwa.vel_limit_min, 
		&rlwa.pos_valid_min, &rlwa.vel_valid_min, &rlwa.pos_limit_min, &rlwa.vel_limit_min, 
		&amc.pos_valid_max,	&amc.vel_valid_max, &amc.pos_limit_max, &amc.vel_limit_max,
		&waist.pos_valid_max,	&waist.vel_valid_max, &waist.pos_limit_max, &waist.vel_limit_max,
		&rlwa.pos_valid_max,	&rlwa.vel_valid_max, &rlwa.pos_limit_max, &rlwa.vel_limit_max,
		&llwa.pos_valid_max,	&llwa.vel_valid_max, &llwa.pos_limit_max, &llwa.vel_limit_max};
	for(size_t i=0; i<16; i++)  { aa_fset(*limits[i],-1024.1, (i < 8) ? 2 : 7); }
	for(size_t i=16; i<32; i++) { aa_fset(*limits[i],1024.1, (i < 24) ? 2 : 7); }

	// Reset motors
	double zeros2[2] = {0.0, 0.0}, zeros7[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
	somatic_motor_cmd(&daemon_cx, &waist, SOMATIC__MOTOR_PARAM__MOTOR_RESET, zeros2, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, zeros7, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, zeros7, 7, NULL);
	
	// Update the motors to get the current values
	somatic_motor_update(&daemon_cx, &amc);
	somatic_motor_update(&daemon_cx, &waist);
	somatic_motor_update(&daemon_cx, &rlwa);
	somatic_motor_update(&daemon_cx, &llwa);
	usleep(1e5);

	// Set the offset values to amc motor group so initial wheel pos readings are zero
	double imu, imuSpeed;
	getImu(&imuChan, imu, imuSpeed, 0.0, NULL); 
	somatic_motor_update(&daemon_cx, &amc);
	double pos_offset[2] = {-amc.pos[0]-imu, -amc.pos[1]-imu};
	aa_fcpy(amc.pos_offset, pos_offset, 2);
	usleep(1e5);

	// Initialize kalman filter for the imu and set the measurement and meas. noise matrices
	// Also, set the initial reading to the current imu reading to stop moving from 0 to current
	kf = new filter_kalman_t;
	filter_kalman_init(kf, 2, 0, 2);
	kf->C[0] = kf->C[3] = 1.0;
	kf->Q[0] = kf->Q[3] = 1e-3;
	kf->x[0] = imu, kf->x[1] = imuSpeed;

	// =======================================================================
	// Create a thread to wait for user input to begin balancing

	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// Send zero velocity to the motors and kill daemon. Also clean up daemon structs.
void destroy() {

	// Close imu channel
	somatic_d_channel_close(&daemon_cx, &imuChan);

	// Stop the motors
	double zeros2[2] = {0.0, 0.0}, zeros7[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
	somatic_motor_cmd(&daemon_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, zeros2, 2, NULL);
	somatic_motor_cmd(&daemon_cx, &waist, SOMATIC__MOTOR_PARAM__MOTOR_HALT, zeros2, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, zeros7, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, zeros7, 7, NULL);

	// Stop and kill the schunk motors
	somatic_motor_destroy(&daemon_cx, &amc);
	somatic_motor_destroy(&daemon_cx, &waist);
	somatic_motor_destroy(&daemon_cx, &llwa);
	somatic_motor_destroy(&daemon_cx, &rlwa);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// Load the world and the robot
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton(0);

	// Fix the gains for the balancing
	K_bal << 250.0, 45.0, 3.0, 10.0, 3.0, 7.0;

	// Read the standing up gains from the command line
	assert(argc == 7 && "Where is my gains for th, x and spin?");
	K_stand << atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]),atof(argv[5]),atof(argv[6]);
	cout << "K_stand: " << K_stand.transpose() << "\nPress enter: " << endl;
	getchar();

	// Initialize, run, destroy
	init();
	run();
	destroy();
	return 0;
}
