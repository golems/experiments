/**
 * @file 08-wheel.cpp
 * @author Can Erdogan
 * @date Jan 03, 2014
 * @brief This executable sends torque commands to the wheels
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

double cur = 0;

/* ******************************************************************************************** */
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "01-balance";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL, &daemon_cx, robot); 
}

/* ******************************************************************************************** */
void destroy () {
	double vals[] = {0.0, 0.0};
	somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, vals, 2, NULL);
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
	somatic_motor_t* amc = krang->amc;
	while(!somatic_sig_received) {

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Read motor encoders, imu and ft and update dart skeleton
		krang->updateSensors(dt);
		cout << "\nwheels: (" << amc->cur[0] << ", " << amc->cur[1] << ")" << endl;

		// Send current command
		double input [] = {cur, cur};
		somatic_motor_cmd(&daemon_cx, amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);

		usleep(1e5);

	}

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// Read the commanded current from the arguments
	if(argc > 1) cur = atof(argv[1]);
	assert(fabs(cur) < 30.0 && "Too much current");

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
