/**
 * @file forwardKinematics.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date April 24, 2012
 * @briefs This file receives the position of the red dot detected from the
 * vision PC and predicts the loaction of the red dot by forward kinematics
 * and then compares the two to determine the accuracy of the forward 
 * kinematics.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

#include <schkin.h>
#include <amino.h>
#include <iostream>

#include "krang.h"
#include "Joystick.h"
#include "Motor.h"
#include "Controllers.h"

/* ********************************************************************************************** */

// Global variables
somatic_d_opts_t krang_d_opts;
krang_cx_t krang_cx;									
extern somatic_d_t krang_d_cx;

/// argp program version
const char *argp_program_version = "forwardKinematics 0.0";
#define ARGP_DESC "verifies forward kinematics for krang"

// The ach channel and its name to receive information from vision PC
ach_channel_t achChannelTransform;
const char *channelNameTransform;

// Argument processing
static int parse_opt( int key, char *arg, struct argp_state *state);
extern struct argp_option argp_options[];
extern struct argp argp;

Joystick *js;
Motor *rlwa, *llwa;

#define RAD2DEG(x) (x*180.0/M_PI)

/* ********************************************************************************************* */
void init() {
	// Set the channel name to receive red dot position
	
	// Create the interfaces for the joystick and schunk modules
	js	= new Joystick("joystick-data");
	rlwa  = new Motor("rlwa-cmd", "rlwa-state", "Right arm", 7);
	llwa  = new Motor("llwa-cmd", "llwa-state", "Left arm", 7);

	// Reset PCIO
	rlwa->reset();
	llwa->reset();

	// ------ daemon init -----------
	somatic_d_opts_t dopt;
	memset(&krang_cx, 0, sizeof(krang_cx)); // zero initialize
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "bal-hack";
	somatic_d_init( &(krang_cx.d_cx), &dopt );

	// --------- open channels ----------
	somatic_d_channel_open( &(krang_cx.d_cx), &(krang_cx.state_chan), "krang-state", NULL);
	channelNameTransform = "chan_transform";
	somatic_d_channel_open(&krang_cx.d_cx, &achChannelTransform, channelNameTransform, NULL);	

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
	// NOTE: This is usually done with SOMATIC_D_GET which is a macro.

	// Set the time to read (?)
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&krang_cx.d_cx, &achChannelTransform, &numBytes, &abstimeout, ACH_O_LAST, &result);


	if(numBytes == 0) return;

	// Return if there is nothing to read
	// =======================================================
	// B. Read message

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(krang_cx.d_cx.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return;
	if(msg->meta->type != SOMATIC__MSG_TYPE__TRANSFORM) return; 

	// Read the force-torque message
	Somatic__Transform* trMessage = somatic__transform__unpack(&(krang_cx.d_cx.pballoc), numBytes, buffer);
	printf("[server] transform:\t");
	for(size_t i = 0; i < 3; i++)
		printf("%6.2f  ", trMessage->translation->data[i]); 
	for(size_t i = 0; i < 4; i++)
		printf("%6.2f  ", trMessage->rotation->data[i]); 
	printf("\n"); fflush(stdout);
}

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {
		// reception of the 3-D coordinate of the red dot from vision PC
		updateRedDot();
		
		// Get the data from the ach channel
		js->update();
		rlwa->update();
		llwa->update();

		// Change to local format
		aa_fcpy(krang_cx.X.arm[KRANG_I_RIGHT].G.q, rlwa->motor.pos, 7);
		aa_fcpy(krang_cx.X.arm[KRANG_I_RIGHT].G.dq, rlwa->motor.vel, 7);
		aa_fcpy(krang_cx.X.arm[KRANG_I_LEFT].G.q, llwa->motor.pos, 7);
		aa_fcpy(krang_cx.X.arm[KRANG_I_LEFT].G.dq, llwa->motor.vel, 7);

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
		rlwa->set_velocity(UR);
		llwa->set_velocity(UL);
	aa_mem_region_release(&krang_cx.d_cx.memreg);	// free buffers allocated during this cycle
	}

	// Send the stoppig event
	somatic_d_event(&krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {
	rlwa->halt();
	llwa->halt();
	delete js;
	delete rlwa;
	delete llwa;

	somatic_d_channel_close(&krang_cx.d_cx, &achChannelTransform);
	somatic_d_destroy(&krang_cx.d_cx);
}

/* ********************************************************************************************* */
int main() {


	init();
	run();
	destroy();

	return 0;
}
