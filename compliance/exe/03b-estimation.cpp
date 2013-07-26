/**
 * @file 03-estimation.cpp
 * @author Can Erdogan
 * @date June 15, 2013
 * @brief This file demonstrates how to estimate the external forces from the ft values by
 * removing the effect of the end-effector in the readings. We remove the drift/bias in
 * the values as shown in 02-correction.cpp.
 * See the FTEstimation report at @thebrain:/home/git/krang/Reports.
 * Note that the F/T readings are in Nm's (Newton-meters).
 */

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include "kore.h"

using namespace std;
using namespace Krang;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
simulation::World* world = NULL;
Hardware* hw;

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	while(!somatic_sig_received) {
		
		// Get the time that has passed
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Update the sensor readings and the kinematics
		c++;
		hw->updateSensors(dt);

		// Print the external f/t readings for the left arm
		if(c % 10 == 1) {
			hw->printState();
			cout << "lft ext: " << hw->lft->lastExternal.transpose() << "\n" << endl;
		}
		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}


/* ******************************************************************************************** */
void init () {

	// Get the world
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "03b-estimation";
	somatic_d_init( &daemon_cx, &dopt );

	// Get the robot
	hw = new Hardware((Hardware::Mode)(Hardware::MODE_ALL & ~Hardware::MODE_GRIPPERS), &daemon_cx, world->getSkeleton("Krang"));
	cout << "Initialization done!" << endl;
}

/* ******************************************************************************************** */
/// The main thread
int main(const int argc, char** argv) {

	init();
	run();
	somatic_d_destroy(&daemon_cx);
	return 0;
}
