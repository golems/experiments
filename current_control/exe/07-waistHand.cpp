/**
 * @file 07-waistHand.cpp
 * @author Can Erdogan
 * @date Jan 03, 2014
 * @brief This executable shows the control of the end-effector force in the z direction with the 
 * current control of the waist motors using a PID loop.
 */

#include <iostream>

#include <Eigen/Dense>

#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <initModules.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>

#include <kore.hpp>

using namespace std;

somatic_d_t daemon_cx;				///< The context of the current daemon
Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
simulation::World* world;			///< the world representation in dart
dynamics::SkeletonDynamics* robot;			///< the robot representation in dart

Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		///< The waist command

double kp = 0.050;
double kd = 0.0;
double ki = 0.0;
double goal = -10.0;

double initWaistAngle;

/* ******************************************************************************************** */
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "01-balance";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL, &daemon_cx, robot); 
	Eigen::Vector2d imuWaist = robot->getConfig(Krang::imuWaist_ids);
	initWaistAngle = imuWaist(1);
	cout << "initWaistAngle: " << initWaistAngle << endl;
	sleep(1);

}

/* ******************************************************************************************** */
void destroy () {
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
	int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)));
	delete krang;
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing data until stop received
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	bool debug = 1; // (c_++ % 20 == 0);
	double lastError = 0.0, totalError = 0.0, firstError = 0.0;
	while(!somatic_sig_received) {


		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Read motor encoders, imu and ft and update dart skeleton
		krang->updateSensors(dt);
		if(debug) cout << "\nrft: " << krang->fts[Krang::RIGHT]->lastExternal.transpose() << endl;
		double curr = krang->fts[Krang::RIGHT]->lastExternal(2);

		// Compute the proportional error
		error = (goal - curr);

		// Compute the current that is needed to apply the 
		if(debug) cout << "error: " << lastError << endl;
		double input = -kp * lastError  - 0.6;
		totalError += lastError;
		double integralinput = -ki * totalError;
		if(debug) cout << "\tinput: " << input << endl;

		// Apply the current
		somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__REAL_CURRENT_MODE);
		somatic_vector_set_data(waistDaemonCmd->data, &input, 1);
		int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
		if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
			ach_result_to_string(static_cast<ach_status_t>(r)));

		// Check if the waist is too far away from its initial position - if so, stop
		Eigen::Vector2d imuWaist = robot->getConfig(Krang::imuWaist_ids);
		double posError = imuWaist(1) - initWaistAngle;
		if(fabs(posError) > 0.30) {
			cout << "moved too much, exitting" << endl;
			destroy();
			exit(0);
		}

		usleep(1e5);

	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// Load the world and the robot
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton(0);

	// Initialize, run, destroy
	init();
	run();
	destroy();
	return 0;
}
