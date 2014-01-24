/**
 * @file 02-position.cpp
 * @author Can Erdogan
 * @date May 23, 2013
 * @brief This demo shows the ability to position the gripper fingers to specific locations
 * and get meaningful position information from the drivers (previous demo just has 0's).
 * To show this, it will (1) open the hand, (2) close the hand, (3) half open it and (4) close it
 * fully.
 * Note 1: For some reason we can only get meaningful updates after sending a motor command
 * and that only works if we are in this daemon mode (need to learn more about somatic!).
 * Note 2: At the moment, although we get nice position control, we are not controlling the velocity
 * with which the fingers move. Although PowerCube gives this option, I am not sure if somatic does.
 */

#include <unistd.h>
#include <iostream>


#include "initModules.h"

using namespace std;

/* ********************************************************************************************** */
somatic_d_t daemon_cx;
ach_channel_t state_chan;
somatic_motor_t lgripper;
double goals [] = {0.058, 0.009, 0.040, 0.009};

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	int currGoal = 0;
	while(!somatic_sig_received) {

		// Set the velocity mode
		double q = goals[currGoal];
		somatic_motor_cmd(&daemon_cx, &lgripper, SOMATIC__MOTOR_PARAM__MOTOR_POSITION, &q, 1, NULL);

		// Get the gripper position
		somatic_motor_update(&daemon_cx, &lgripper);
		printf("Current goal: %lf (%d), pos: %lf\n", q, currGoal, lgripper.pos[0]); 
		fflush(stdout);

		// Set the next goal if reached the current one
		if(fabs(lgripper.pos[0] - goals[currGoal]) < 1e-4) {
			currGoal = (currGoal + 1) % 4;
			cout << "Changing goal!!!" << endl;
		}

		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
		
		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void init () {

	// Initialize a daemon context
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "position";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the gripper
	initGripper(daemon_cx, lgripper, "lgripper");
}

/* ********************************************************************************************* */
void destroy() {
	somatic_motor_cmd(&daemon_cx, &lgripper, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
int main() {

	init();
	run();
	destroy();

	return 0;
}
