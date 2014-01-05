/**
 * @file 05-external.cpp
 * @author Can Erdogan
 * @date Jan 05, 2014
 * @brief This executable shows the external f/t values.
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
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing data until stop received
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	while(!somatic_sig_received) {

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Read motor encoders, imu and ft and update dart skeleton
		krang->updateSensors(dt);
		cout << "rft: " << krang->fts[Krang::RIGHT]->lastExternal.transpose() << endl;
		usleep(1e5);

	}

	// Send the stopping event
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

	// Initialize the daemon and the drivers
	init();
	
	// Print the f/t values
	run();

	// Destroy the daemon and the robot
	somatic_d_destroy(&daemon_cx);
	delete krang;
	return 0;
}
