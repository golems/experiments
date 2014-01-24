/**
 * @file 03-grasp.cpp
 * @author Can Erdogan
 * @date May 23, 2013
 * @brief This demo shows the ability to grasp an object with velocity control. The idea is that
 * we will slowly close the hand until the change in the position is minimal. Note that for this,
 * the object that is being held should be a hard material (definitely no hands!).
 */

#include <unistd.h>
#include <iostream>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

using namespace std;

/* ********************************************************************************************** */
somatic_d_t daemon_cx;
ach_channel_t state_chan;
somatic_motor_t lgripper;

/* ********************************************************************************************* */
double lastPos = 0.0;
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	bool grasped = false;
	while(!somatic_sig_received) {

		// Set the velocity mode
		static int i = 0;
		double dq = grasped ? 0.0 : -0.005;
		somatic_motor_cmd(&daemon_cx, &lgripper, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, &dq, 1, NULL);

		// Get the gripper position
		somatic_motor_update(&daemon_cx, &lgripper);
		printf("i: %d, pos: %lf, lastPos: %lf, grasped: %d, dq: %lf\n", i, lgripper.pos[0], lastPos, grasped, dq); 
		fflush(stdout);

		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
		
		// Check if the grasp closed
		if((i > 100) && (fabs(lastPos - lgripper.pos[0]) < 1e-5))
			grasped = true;

		// Sleep and set the last position
		if(i++ % 5 == 4) lastPos = lgripper.pos[0];
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

	// Create the motor reference for the left gripper
	somatic_motor_init(&daemon_cx, &lgripper, 1, "lgripper-cmd", "lgripper-state");
	usleep(1e5);

	// Set the min/max values for valid and limits values
	aa_fset(lgripper.pos_valid_min, 0.009, 1);
	aa_fset(lgripper.pos_limit_min, 0.009, 1);
	aa_fset(lgripper.pos_valid_max, 0.068, 1);
	aa_fset(lgripper.pos_limit_max, 0.068, 1);
	aa_fset(lgripper.vel_valid_min, -0.018, 1);
	aa_fset(lgripper.vel_limit_min, -0.018, 1);
	aa_fset(lgripper.vel_valid_max, 0.018, 1);
	aa_fset(lgripper.vel_limit_max, 0.018, 1);

	// Update and reset them
	somatic_motor_update(&daemon_cx, &lgripper);
	somatic_motor_cmd(&daemon_cx, &lgripper, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
	lastPos = lgripper.pos[0];
	usleep(1e5);

	// Open the state channel
//	somatic_d_channel_open(&daemon_cx, &state_chan, "position-state", NULL);
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
