/**
 * @file 04-balancing.cpp
 * @author Munzir Zafar, Can Erdogan
 * @date July 11, 2013
 * @brief This demonstration shows the balancing of the robot while following the position
 * and velocity commands set by the joystick.
 * NOTE We are using a Vector6d to represent the state which has: th, th. x, x., psi, psi.
 * where . is the derivative, th is the imu angle, x is the distance along the heading and
 * psi is the heading angle.
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
// Constants for the robot kinematics

const double wheelRadius = 10.5; 							///< Radius of krang wheels in inches
const double distanceBetweenWheels = 27.375; 	///< Distance Between krang's wheels in inches 

/* ******************************************************************************************** */
// Globals for imu, motors and joystick

filter_kalman_t *kf;					///< the kalman filter to smooth the imu readings
ach_channel_t imuChan;				///< the state channel to listen to imu data
somatic_d_t daemon_cx;				///< The properties of this "daemon"
somatic_motor_t amc; 					///< The interface to the wheel motor group
somatic_motor_t waist; 				///< The interface to the waist motor group
somatic_motor_t llwa; 				///< The interface to the llwa motor group
somatic_motor_t rlwa; 				///< The interface to the rlwa motor group
ach_channel_t js_chan;				///< The ach channel to the joystick daemon

simulation::World* world;			///< the world representation in dart
SkeletonDynamics* robot;			///< the robot representation in dart

bool start = false;						///< Giving time to the user to get the robot in balancing angle
Vector6d K;										///< The gains for the controller

//SkeletonDynamics* robot;			///< the robot representation in dart
typedef Matrix <double, 6, 1> Vector6d;		///< The state variable

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

	// Set the values for the axis
	double* x = &(js_msg->axes->data[0]);
	js_forw = -x[1];
	js_spin = 0.0; //x[2];

	// Change the gains with the given joystick input
	double deltaTH = 0.2;					// deltaX = 0.02;
	int64_t* b = &(js_msg->buttons->data[0]);
	for(size_t i = 0; i < 4; i++)
		if(b[i] == 1)
			K(i % 2) += ((i < 2) ? deltaTH : -deltaTH);
	
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
		if(debug) printf("K: <%7.3lf, %7.3lf, %7.3lf, %7.3lf\n", K(0), K(1), K(2), K(3));

		// Determine the reference values for x and psi
		updateReference(js_forw, js_spin, dt, refState);
		if(debug) cout << "refState: " << refState.transpose() << endl;
		if(debug) cout << "state: " << state.transpose() << endl;
		
		// Compute the error term between reference and current, and weight with gains (spin separate)
		Vector6d error = state - refState;
		double u = K.topLeftCorner<4,1>().dot(error.topLeftCorner<4,1>());
		double u_spin = -K.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());			// - on purpose here 
		if(debug) printf("u: %lf, u_spin: %lf, uL: %lf, uR: %lf\n", u, u_spin, u + u_spin, u - u_spin);

		// Compute the input for left and right wheels
		double input [2] = {u + u_spin, u - u_spin};

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

	// Read the gains from the command line
	assert(argc == 7 && "Where is my gains for th, x and spin?");
	K << atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]);
	cout << "K: " << K.transpose() << "\nPress enter: " << endl;
	getchar();

	// Initialize, run, destroy
	init();
	run();
	destroy();
	return 0;
}
