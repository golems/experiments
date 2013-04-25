/**
 * @file forwardKinematics.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date April 24, 2012
 * @briefs This file receives the position of the red dot detected from the
 * vision PC and predicts the loaction of the red dot by forward kinematics
 * and then compares the two to determine the accuracy of the forward 
 * kinematics.
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

#include <schkin.h>
#include <amino.h>
#include <iostream>

#include "krang.h"
#include "Joystick.h"
#include "Controllers.h"

/* ********************************************************************************************** */

krang_cx_t krang_cx;									

// The ach channel and its name to receive information from vision PC
ach_channel_t chan_transform;

Joystick *js;
somatic_motor_t llwa, rlwa;

/* ********************************************************************************************* */
void init() {
	
	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&krang_cx, 0, sizeof(krang_cx)); // zero initialize
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "bal-hack";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the motors with the daemon context, channel names and # of motors
	somatic_motor_init(&daemon_cx, &llwa, 7, "llwa-cmd", "llwa-state");
	somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&llwa.pos_valid_min, &rlwa.pos_valid_min, &llwa.vel_valid_min, &rlwa.vel_valid_min, 
		&llwa.pos_limit_min, &rlwa.pos_limit_min, &llwa.pos_limit_min, &rlwa.pos_limit_min, 
		&llwa.pos_valid_max, &rlwa.pos_valid_max, &llwa.vel_valid_max, &rlwa.vel_valid_max, 
		&llwa.pos_limit_max, &rlwa.pos_limit_max, &llwa.pos_limit_max, &rlwa.pos_limit_max};
	for(size_t i = 0; i < 8; i++) aa_fset(*limits[i], -1024.1, 7);
	for(size_t i = 8; i < 16; i++) aa_fset(*limits[i], 1024.1, 7);
	
	// Update and reset them
	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
	somatic_motor_update(&daemon_cx, &rlwa);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);

	// Create the interfaces for the joystick and schunk modules
	js	= new Joystick("joystick-data");

	// --------- open channels ----------
	somatic_d_channel_open( &daemon_cx, &(krang_cx.state_chan), "krang-state", NULL);
	somatic_d_channel_open(&daemon_cx, &chan_transform, "chan_transform", NULL);	

	// --------- init arm controller ----------------
	for( size_t i = 0; i < 2; i ++ ) {
		rfx_ctrl_ws_init( &krang_cx.X.arm[i].G, 7 );
		rfx_ctrl_ws_lin_k_init( &krang_cx.X.arm[i].K, 7 );
	}

}

/* ********************************************************************************************* */
void updateRedDot() {

	// =======================================================
	// A. Get message

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(1));
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &chan_transform, &numBytes, &abstimeout, 
		ACH_O_LAST, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return;

	// =======================================================
	// B. Read message

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return;
	if(msg->meta->type != SOMATIC__MSG_TYPE__TRANSFORM) return; 

	// Read the force-torque message
	Somatic__Transform* message = somatic__transform__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	printf("[server] transform:\t");
	for(size_t i = 0; i < 3; i++)
		printf("%6.2f  ", message->translation->data[i]); 
	for(size_t i = 0; i < 4; i++)
		printf("%6.2f  ", message->rotation->data[i]); 
	printf("\n"); fflush(stdout);
}

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Receive the 3-D coordinate of the red dot from vision PC
		updateRedDot();
		
		// Get the data from the ach channel
		js->update();
		somatic_motor_update(&daemon_cx, &llwa);
		somatic_motor_update(&daemon_cx, &rlwa);

		// Set the Krang state of the context, the joystick values, f/t and workspace 
		// control from joystick 
		memcpy(&krang_cx.ui, &js->jsvals, sizeof(krang_cx.ui));
		
		// Change the control mode based on the joystick and set the reference velocity arm/wheel vels.
		Joystick::process_input(&krang_cx, 0.0);

		// Get the joint torques based on the mode
		double UR[7] = {0};
		double UL[7] = {0};
		Controllers::arm(&krang_cx, 0.0, UR, UL);

		// Set the velocities
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, UL, 7, NULL);
		somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, UR, 7, NULL);

		// free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	delete js;

	somatic_d_channel_close(&daemon_cx, &chan_transform);
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
int main() {


	init();
	run();
	destroy();

	return 0;
}
