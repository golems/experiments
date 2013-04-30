#include <somatic.h>
#include <somatic/daemon.h>
#include "krang.h"
#include "krang-io.h"
#include <schkin.h>
#include <amino.h>
#include "Controllers.h"
#include <iostream>
///////////////////////////////////////////////////////////////////////////////////////////////////
 // Global variables

somatic_d_opts_t krang_d_opts;
krang_cx_t krang_cx;									
extern somatic_d_t krang_d_cx;

Krang * io;
#define RAD2DEG(x) (x*180.0/M_PI)



///////////////////////////////////////////////////////////////////////////////////////////////////
void init() {
	// ------ daemon init -----------
	somatic_d_opts_t dopt;
	memset(&krang_cx, 0, sizeof(krang_cx)); // zero initialize
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "balancing-experiment";
	somatic_d_init( &(krang_cx.d_cx), &dopt );

	// --------- open channels ----------
	somatic_d_channel_open( &(krang_cx.d_cx),
							&(krang_cx.state_chan), "krang-state",
							NULL );
	// --------- init parse table ----------
	// set initial mode
	krang_parse_init(krang_cx.parse_table);

	krang_cx.X.mode = KRANG_MODE_HALT;


	io = new Krang(&(krang_cx.X));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main loop that interfaces with the I/O from the joystick.
void run() {
	// Send the event massage
	somatic_d_event( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_RUNNING,
					 NULL, NULL );


	// Set the timestep to update
	double dt = 0.0;		

	// Get the time in the beginning
	struct timespec t_now, t_prev;
	t_prev = aa_tm_now();

	static int c_ = 0;
	while (!somatic_sig_received ) {
		bool debug = 0;
		// ===============================================================
		// Data retrieval

		// Get the current time and compute the time difference
		t_now = aa_tm_now();						
		dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	

		// Read ach channels for imu, wheels and waist pos/vel and filter the data 
		io->update_and_filter();

		// control from joystick 
		io->get_js(&krang_cx.ui); 
		
		// Evaluate the current imu and torso readings and generate thresh event
		krang_threshold(&krang_cx);

		// Change the control mode based on the joystick and set the reference velocity arm/wheel vels.
		Joystick::process_input(&krang_cx, dt);

		// ===============================================================
		// Wheel control

		bool SKIP_AMC=0;
		double amc_current[2];
		bool SKIP_MOTORS = 0;
		if(!SKIP_MOTORS) {

			// Switch on the balancing mode
			double amc_current[2];
			switch(krang_cx.X.mode) {
				case KRANG_MODE_HALT: {
					krang_cx.X.q1_ref[0] = krang_cx.X.q1_0;
					krang_cx.X.q1_ref[1] = krang_cx.X.q1_1;
					krang_cx.X.dq1_ref[0] = 0;
					krang_cx.X.dq1_ref[1] = 0;
				}	break;
				case KRANG_MODE_SIT_LO:
				case KRANG_MODE_SIT_HI: {
					Controllers::insit( amc_current, &krang_cx.X);
					if(!SKIP_AMC) io->amc->set_current(amc_current);
				}	break;
				case KRANG_MODE_TOSIT: {
					Controllers::tosit( amc_current, &krang_cx.X);
					if(!SKIP_AMC) io->amc->set_current(amc_current);
				}	break;
				case KRANG_MODE_BALANCE_LO:
				case KRANG_MODE_BALANCE_HI: {
					static bool override=0;
					if(!override) { 
						printf("Setting the override mode!\n");
						io->amc->digital_out(19,1); 
						override=1; 
					}

					// Balancing mode:
					Controllers::balance(amc_current, &krang_cx.X);
					if(!SKIP_AMC) io->amc->set_current(amc_current);
				}	break;
				case KRANG_MODE_QUIT:
				case KRANG_MODE_BAD:
				case KRANG_MODE_SIZE:
				default: 
					somatic_sig_received = 1;
			}	// enbd of balancing switch
		}	// end of motor control if statement

		// Update previous measured time
		t_prev = t_now;

		// Release memory
		aa_mem_region_release( &krang_cx.d_cx.memreg );
	}
	// Once the main loop returns, the program is done. Send the stopping
	// event message.
	somatic_d_event( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING,
					 NULL, NULL );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void destroy() {
	// Clean up
	io->amc->digital_out(19,0);
	delete io;
	somatic_d_destroy( &krang_cx.d_cx );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// The main function
int main() {
	init();
	run();
	destroy();
	return 0;
}
