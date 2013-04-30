#include <somatic.h>
#include <somatic/daemon.h>
#include "krang.h"
#include "krang-io.h"
#include <schkin.h>
#include <amino.h>
#include "Controllers.h"
#include <iostream>
/* ********************************************************************************************* */
// Global variables
somatic_d_opts_t krang_d_opts;
krang_cx_t krang_cx;									
extern somatic_d_t krang_d_cx;
krang_state_t state;
Joystick *js;
somatic_motor_t amc, waist; 
Imu *imu;
filter_kalman_t *kf;
#define RAD2DEG(x) (x*180.0/M_PI)

/* ********************************************************************************************* */
void init() {
	// Initialize the program
	somatic_d_opts_t dopt;
	memset(&krang_cx, 0, sizeof(krang_cx)); // zero initialize
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "balancing-experiment";
	somatic_d_init( &(krang_cx.d_cx), &dopt );

	// Initialize the mode/event table
	krang_parse_init(krang_cx.parse_table);

	// Set initial mode
	state.mode = KRANG_MODE_HALT;

	// Initialize the motors with daemon context, channel names and # of motors
	somatic_motor_init( &krang_cx.d_cx, &amc, 2, "amc-cmd", "amc-state");
	somatic_motor_init( &krang_cx.d_cx, &waist, 2, "waist-cmd", "waist-state");

	// Set the min and maximum position and velocity valid/limit values for motors
	double ** limits[] = { 
		&amc.pos_valid_min, &amc.vel_valid_min, &amc.pos_limit_min, &amc.vel_limit_min, 
		&waist.vel_valid_max, &waist.pos_limit_max, &waist.vel_limit_max,  &amc.pos_valid_max,
		&amc.vel_valid_max, &amc.pos_limit_max, &amc.vel_limit_max, &waist.vel_valid_max, 
		&waist.pos_limit_max, &waist.vel_limit_max};
	for(size_t i=0; i<7; i++) { aa_fset(*limits[i],-1024.1, 2); }
	for(size_t i=7; i<14; i++) { aa_fset(*limits[i],1024.1, 2); }
	aa_fset( waist.pos_valid_min, -5, 2);
	aa_fset( waist.pos_valid_max, 5, 2);
	usleep(1e5);

	// Update and reset motors
	somatic_motor_update(&krang_cx.d_cx, &waist);
	somatic_motor_cmd(&krang_cx.d_cx, &waist, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 2, NULL);
	somatic_motor_update(&krang_cx.d_cx, &amc);
	somatic_motor_cmd(&krang_cx.d_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 2, NULL);
	
	// Check waist positions
	if (fabs(waist.pos[0] + waist.pos[1]) > 2e-2)
		fprintf(stderr, "ERROR: Waist position mismatched: (%f, %f)\n", waist.pos[0], -waist.pos[1]);

	imu   = new Imu("imu-data", 0.0);
	js	= new Joystick("joystick-data");
	// Open the state channel
	somatic_d_channel_open( &(krang_cx.d_cx),
							&(krang_cx.state_chan), "krang-state",
							NULL );
	// ==============================================================================================
	// Initialize the kalman filter
	kf = new filter_kalman_t;
	filter_kalman_init( kf , 8 , 0 , 8 );
	double T = 2.04 * 1e-3;  // second
	memcpy(kf->x, kf->z, sizeof(double)*8);
	// Process matrix - fill every 9th value to 1 and every 18th starting from 8 to T.
	for(size_t i = 0; i < 64; i += 9)
		kf->A[i] = 1.0;
	for(size_t i = 8; i < 64; i += 18)
		kf->A[i] = T;
	// Process noise covariance matrix
	const double k1 = 2.0;
	const double k1b = 5.0;
	const double k2 = 10.0;
	const double k3 = 1.0;
	kf->R[0] = (T*T*T*T)*k1*(1.0/4.0);
	kf->R[1] = (T*T*T)*k1*(1.0/2.0);
	kf->R[8] = (T*T*T)*k1*(1.0/2.0);
	kf->R[9] = (T*T)*k1b;
	kf->R[18] = (T*T*T*T)*k2*(1.0/4.0);
	kf->R[19] = (T*T*T)*k2*(1.0/2.0);
	kf->R[26] = (T*T*T)*k2*(1.0/2.0);
	kf->R[27] = (T*T)*k2;
	kf->R[36] = (T*T*T*T)*k2*(1.0/4.0);
	kf->R[37] = (T*T*T)*k2*(1.0/2.0);
	kf->R[44] = (T*T*T)*k2*(1.0/2.0);
	kf->R[45] = (T*T)*k2;
	kf->R[54] = (T*T*T*T)*k3*(1.0/4.0);
	kf->R[55] = (T*T*T)*k3*(1.0/2.0);
	kf->R[62] = (T*T*T)*k3*(1.0/2.0);
	kf->R[63] = (T*T)*k3;
	// Measurement matrix - fill every 9th value to 1
	for(size_t i = 0; i < 64; i += 9)
		kf->C[i] = 1.0;
	// Measurement noise covariance matrix
	double imuCov = 1e-3;	//1e-3
	kf->Q[0] = imuCov;	// IMU
	kf->Q[9] = imuCov;
	kf->Q[18] = 0.0005; // AMC
	kf->Q[27] = 0.02;
	kf->Q[36] = 0.0005; // AMC
	kf->Q[45] = 0.02;
	kf->Q[54] = 0.05;   // Torso
	kf->Q[63] = 0.001;
	// ==============================================================================================
}

/* ********************************************************************************************* */
void update() {
	// Update all the devices connected
	js->update();
	imu->update();
	somatic_motor_update(&krang_cx.d_cx, &amc);
	somatic_motor_update(&krang_cx.d_cx, &waist);
}

/* ********************************************************************************************* */
void filter() {
	// Set up the kalman filter with the raw data for imu, waist and amc
	// NOTE: The arm values are not filtered. Why not?
	kf->z[0] = imu->get_th();  // = q2
	kf->z[1] = imu->get_dth(); // = dq2
	kf->z[2] = amc.pos[0]+imu->get_th();  // = abs. L wheel pos.
	kf->z[3] = amc.vel[1]+imu->get_dth(); // = abs. L wheel vel.
	kf->z[4] = amc.pos[1]+imu->get_th();  // = abs. R wheel pos.
	kf->z[5] = amc.vel[1]+imu->get_dth(); // = abs. R wheel vel.
	kf->z[6] = waist.pos[0];
	kf->z[7] = waist.vel[0];
	// Call the kalman filter
	filter_kalman_predict( kf );
	filter_kalman_correct( kf );
	// Set the filtered values to the local fields
	state.q2		  = kf->x[0];
	state.dq2		 = kf->x[1];
	state.q1_0		= kf->x[2] - state.q2;
	state.dq1_0	   = kf->x[3] - state.dq2;
	state.q1_1		= kf->x[4] - state.q2;
	state.dq1_1	   = kf->x[5] - state.dq2;
	state.q3		  = kf->x[6];
	state.dq3		 = kf->x[7];
	// Set the wheel position by taking the mean of the two wheel encoders 
	state.q1 = (state.q1_0 + state.q1_1)/2.0;
	state.dq1 = (state.dq1_0 + state.dq1_1)/2.0;
}

/* ********************************************************************************************* */
// Read imu angle and speed, wheel position and speed and waist position and speed and filter
// the data
void readSensors() {
	update();
	filter();
}

/* ********************************************************************************************* */
// Read and process the commands from the joystick
void readJoystick( double dt ) {
	// control from joystick 
	memcpy(&krang_cx.ui, &js->jsvals, sizeof(krang_cx.ui));
	// Evaluate the current imu and torso readings and generate thresh event
	krang_threshold(&krang_cx);
	// Change the control mode based on the joystick and set the reference velocity arm/wheel vels.
	Joystick::process_input(&krang_cx, dt);
}

/* ********************************************************************************************* */
// Control the wheels based on the current mode
void controlWheels() {
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
				if(!SKIP_AMC) somatic_motor_cmd(&krang_cx.d_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, amc_current, 2, NULL);;
			}	break;
			case KRANG_MODE_TOSIT: {
				Controllers::tosit( amc_current, &krang_cx.X);
				if(!SKIP_AMC) somatic_motor_cmd(&krang_cx.d_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, amc_current, 2, NULL);;
			}	break;
			case KRANG_MODE_BALANCE_LO:
			case KRANG_MODE_BALANCE_HI: {
				static bool override=0;
				if(!override) { 
					printf("Setting the override mode!\n");
					somatic_motor_digital_out(&krang_cx.d_cx, &amc, 19 ,1);
					override=1; 
				}
				// Balancing mode:
				Controllers::balance(amc_current, &krang_cx.X);
				if(!SKIP_AMC) somatic_motor_cmd(&krang_cx.d_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, amc_current, 2, NULL);;
			}	break;
			case KRANG_MODE_QUIT:
			case KRANG_MODE_BAD:
			case KRANG_MODE_SIZE:
			default: 
				somatic_sig_received = 1;
		}	// enbd of balancing switch
	}	// end of motor control if statement
}

/* ********************************************************************************************* */
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
	while (!somatic_sig_received ) {
		// Get the current time and compute the time difference
		t_now = aa_tm_now();						
		dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		// Get wheel, imu, waist positions and velocities from ack chennels
		readSensors();
		// Get commands from the joystick and process them
		readJoystick(dt);
		// Based on the sensor feedback, joystick commands and current state, call the relevant 
		// controller for controlling the wheels
		controlWheels();
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

/* ********************************************************************************************* */
void destroy() {
	delete js;
	delete imu;
	somatic_motor_digital_out(&krang_cx.d_cx, &amc, 19, 0);
	somatic_motor_cmd(&krang_cx.d_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 2, NULL);
	somatic_motor_cmd(&krang_cx.d_cx, &waist, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 2, NULL);
filter_kalman_destroy( kf );
	delete( kf );
	somatic_d_destroy( &krang_cx.d_cx );
}

/* ********************************************************************************************* */
int main() {
	init();
	run();
	destroy();
	return 0;
}
