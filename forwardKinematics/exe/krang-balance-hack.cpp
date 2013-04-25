#include <somatic.h>
#include <somatic/daemon.h>
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

Joystick *js;
Motor *rlwa, *llwa;

#define RAD2DEG(x) (x*180.0/M_PI)

/* ********************************************************************************************** */
void init () {

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

	// --------- init arm controller ----------------
	for( size_t i = 0; i < 2; i ++ ) {
		rfx_ctrl_ws_init( &krang_cx.X.arm[i].G, 7 );
		rfx_ctrl_ws_lin_k_init( &krang_cx.X.arm[i].K, 7 );
	}

}

/* ********************************************************************************************** */
// This is the main loop that interfaces with the I/O from the joystick.
void main_loop() {

	static int c_ = 0;
	while (!somatic_sig_received ) {

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

		// Release memory
		aa_mem_region_release( &krang_cx.d_cx.memreg );
		c_++;
	}
}

void destroy() {
	rlwa->halt();
	llwa->halt();
	delete js;
	delete rlwa;
	delete llwa;
}

/// The main function
int main() {

	// daemon setup
	init();

	// Send the event massage
	somatic_d_event( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_RUNNING,
					 NULL, NULL );

	// The robot is going to be manipulated with the joystick, so start
	// the main loop that interface with the IO.
	main_loop();

	// Once the main loop returns, the program is done. Send the stopping
	// event message.
	somatic_d_event( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING,
					 NULL, NULL );

	// Clean up
	destroy();
	somatic_d_destroy( &krang_cx.d_cx );

	return EXIT_SUCCESS;
}
