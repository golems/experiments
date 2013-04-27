/**
 * @file checkCoM.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date Feb 12, 2013
 * @brief This executable is used to move the robot into a balancing position and print out the
 * reference imu balancing angle (and its error).
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include "krang.h"
#include "krang-io.h"
#include <schkin.h>
#include <amino.h>
#include "Controllers.h"
#include "Dynamics.h"
#include <Eigen/Dense>

#define RAD2DEG(x) ((x)/M_PI*180.0)

/* ********************************************************************************************** */
 // Global variables

somatic_d_opts_t krang_d_opts;
krang_cx_t krang_cx;									
extern somatic_d_t krang_d_cx;

/* ********************************************************************************************** */
// This is the main loop that interfaces with the I/O from the joystick.
void main_loop(Krang *io) {

	// Set the timestep to update
	double dt = 0.0;		

	// Get the time in the beginning
	struct timespec t_now, t_prev;
	t_prev = aa_tm_now();
	
	printf(">> CAUTION: Control loop is active.\n\n");
	static int c_ = 0;
	while (!somatic_sig_received ) {

		bool debug = 1;
		debug &= (c_++ % 500 == 1);

		// ===============================================================
		// Data retrieval

		// Get the current time and compute the time difference
		t_now = aa_tm_now();						
		dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	

		// Get the initial data
		io->update_and_filter();

		// Set the Krang state of the context, the joystick values, f/t and workspace 
		// control from joystick 
		io->get_state(&krang_cx.X);
		io->get_js(&krang_cx.ui); 
		krang_poll(&krang_cx);

		// ===============================================================
		// Set discrete mode and continuous joint values

		// Evaluate the current imu and torso readings and set the current mode 
		krang_threshold(&krang_cx);

		// Change the control mode based on the joystick and set the reference velocity arm/wheel vels.
		Joystick::process_input(&krang_cx, dt);

		// Update the end effector position/orientation and Jacobians and set in the rflx struct.	
		Controllers::kinematics(&krang_cx.X);

		// ===============================================================
		// Get the current CoM, print the error

		// Print the waist and imu values
		if(debug) printf("\nimu: %.3lf, waist: %.3lf\n", RAD2DEG(krang_cx.X.q2), RAD2DEG(krang_cx.X.q3));

		// Get the current com
		static Dynamics dynamics (TOP_LEVEL_PATH"/data/MassProp.table", true);
		Eigen::Vector3d com = dynamics.com(krang_cx.X.q2, krang_cx.X.q3, 0.0, Eigen::VectorXd(), Eigen::VectorXd());
		if(debug) printf("com: %.3lf, %.3lf, %.3lf\n", com(0), com(1), com(2));

		// Compute the error term
		double error = atan2(com(0), com(1));
		if(debug) printf("error: %.3lf\n", RAD2DEG(error));
		
		// ===============================================================
		// Clean up and updates

		// Update previous measured time
		t_prev = t_now;

		// Send the current state for recording purposes (?)
		krang_send_state(&krang_cx);

		// Release memory
		aa_mem_region_release( &krang_cx.d_cx.memreg );
	}
}

/// The main function
int main() {

	// daemon setup
	krang_init(&krang_cx);
	Krang *io = new Krang(&(krang_cx.X));

	// Tell the user how to start the motion	
	fprintf(stderr, ">> Press button [7] & [8] to activate...\n\n");

	// Send the event massage
	somatic_d_event( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_RUNNING,
					 NULL, NULL );

	// The robot is going to be manipulated with the joystick, so start
	// the main loop that interface with the IO.
	main_loop(io);

	// Once the main loop returns, the program is done. Send the stopping
	// event message.
	somatic_d_event( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING,
					 NULL, NULL );

	// Clean up
	delete io;
	somatic_d_destroy( &krang_cx.d_cx );

	return EXIT_SUCCESS;
}
