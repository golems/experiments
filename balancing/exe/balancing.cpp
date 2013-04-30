#include <somatic.h>
#include <somatic/daemon.h>
#include <schkin.h>
#include <amino.h>
#include "Dynamics.h"
#include <Eigen/Dense>
#include <iostream>
#include <unistd.h>
#include <string>
#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <ach.h>
#include <somatic/util.h>
#include <somatic/motor.h>
#include <somatic.pb-c.h>
#include <imud.h>
#include <pciod.h>
#include <filter.h>
#include <krang.h>
#include "krang.h"


/* ********************************************************************************************* */
// Global variables
somatic_d_opts_t krang_d_opts;
krang_cx_t krang_cx;									
extern somatic_d_t krang_d_cx;
krang_state_t state;
somatic_motor_t amc, waist; 
ach_channel_t imu_chan, js_chan;
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

	// Open the IMU channel
	int r  = ach_open( &imu_chan, "imu-data" , NULL );
	aa_hard_assert(r == ACH_OK, "Ach failure %s on opening IMU channel (%s, line %d)\n",
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open Joystick channel
	r  = ach_open(&js_chan, "js-data" , NULL);
	aa_hard_assert(r == ACH_OK,
				   "Ach failure %s on opening Joystick channel (%s, line %d)\n",
				   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
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
void readSensors() {
	
	// --------------------------------------------------------------------
	// Read IMU position and speed
	int r;
	struct timespec abstime = aa_tm_future( aa_tm_sec2timespec( 1.0 / 30.0 ));
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imu_chan, &abstime );

	// Assert that there were no errors in the call
	aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME, 
			"Ach wait failure %s on IMU data receive (%s, line %d)\n",
			 ach_result_to_string(static_cast<ach_status_t>(r)),  __FILE__, __LINE__);

	// If a message was received, extract the pitch and its rate
	if (r == ACH_OK) {

		// Prepare the ssdmu structure 
		ssdmu_sample_t imu_sample;
		imu_sample.x  = imu_msg->data[0];
		imu_sample.y  = imu_msg->data[1];
		imu_sample.z  = imu_msg->data[2];
		imu_sample.dP = imu_msg->data[3];
		imu_sample.dQ = imu_msg->data[4];
		imu_sample.dR = imu_msg->data[5];

		// Make the calls to extract the pitch and rate of extraction
		kf->z[0]  = ssdmu_pitch(&imu_sample);				 
		kf->z[1] = ssdmu_d_pitch(&imu_sample);			

		// Free the unpacked message
		somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );
	}
	// --------------------------------------------------------------------
	// Read wheel position and speed
	somatic_motor_update(&krang_cx.d_cx, &amc);
	kf->z[2] = amc.pos[0]+kf->z[0];  // = abs. L wheel pos.
	kf->z[3] = amc.vel[0]+kf->z[1]; // = abs. L wheel vel.
	kf->z[4] = amc.pos[1]+kf->z[0];  // = abs. R wheel pos.
	kf->z[5] = amc.vel[1]+kf->z[1]; // = abs. R wheel vel.
	// --------------------------------------------------------------------
	// Read waist position and speed
	somatic_motor_update(&krang_cx.d_cx, &waist);
	kf->z[6] = waist.pos[0];
	kf->z[7] = waist.vel[0];
	// --------------------------------------------------------------------
	// Filter the noise	
	filter_kalman_predict( kf );
	filter_kalman_correct( kf );
	// --------------------------------------------------------------------
	// Save the output of the filter in the state structure
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
	// --------------------------------------------------------------------
	// If IMU angle is below the sitting angle, change the mode from TOSIT to SIT
	double imu = state.q2, epsilon=.001;
	if(imu < (SITTING_ANGLE - epsilon)) { if(state.mode == KRANG_MODE_TOSIT) state.mode = KRANG_MODE_SIT; }
}

/* ********************************************************************************************* */
// Read and process the commands from the joystick
void readJoystick( double dt ) {
	
	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

	// Get the values
	char b [10];
	double x [6];
	for(size_t i = 0; i < 10; i++) 
		b[i] = js_msg->buttons->data[i] ? 1 : 0;
	memcpy(x, js_msg->axes->data, sizeof(x));
	
	// Set the forward/backward ad left/right joystick axes values to zero
	// FIXME: This may not be needed
	state.js_fb = 0;	state.js_lr = 0;

	// Check for the start, quit, sit and stand conditions
	// Start Command
	if(b[6] && b[7]) { if(state.mode == KRANG_MODE_HALT) state.mode=KRANG_MODE_SIT; }
	// Quit Command
	else if(b[8]) { state.mode = KRANG_MODE_QUIT; }
	// Sit Command
	else if(b[9] && !b[5]) { if(state.mode == KRANG_MODE_BALANCE) state.mode = KRANG_MODE_TOSIT; } 
	// Stand Command
	else if(b[9] && b[5]) { if(state.mode == KRANG_MODE_SIT) state.mode = KRANG_MODE_BALANCE; }
	// Check for the wheel control - no shoulder button should be pressed
	// None of the shoulder buttons should be pressed
	else if(!b[4] && !b[5] && !b[6] && !b[7]) {
		state.js_fb = -x[1];	// range [-1, 1]
		state.js_lr =  x[2];	// range [-1, 1]
		
    // Velocity control when sitting
    state.dq1_ref[0] = MAX_LIN_VEL*state.js_fb;
    state.dq1_ref[1] = state.dq1_ref[0];

    // Moving left/right: Velocity control for heading
    double max_ang_vel = 2.0;
    state.dq1_ref[0] += max_ang_vel*state.js_lr;
    state.dq1_ref[1] -= max_ang_vel*state.js_lr;

    // integrate
    state.q1_ref[0] += dt * state.dq1_ref[0];
    state.q1_ref[1] += dt * state.dq1_ref[1];
	} 
}

/* ********************************************************************************************* */
// Control the wheels based on the current mode
void controlWheels() {
	bool SKIP_AMC=0;
	double amc_current[2];
	switch(state.mode) {
		// --------------------------------------------------------------------------------------------
		// Halt Mode: Fix the wheel reference pos/vel to current pos/vel
		case KRANG_MODE_HALT: {
			state.q1_ref[0] = state.q1_0;
			state.q1_ref[1] = state.q1_1;
			state.dq1_ref[0] = 0;
			state.dq1_ref[1] = 0;
		}	break;
		// --------------------------------------------------------------------------------------------
		// Sit Mode: Only control the wheel pos/vel (and not imu pos/vel)
		case KRANG_MODE_SIT: {
			double Kp = 3;
			double Kv = 20;
			amc_current[0] = -Kv*(state.dq1_0 - state.dq1_ref[0] + Kp*(state.q1_0 - state.q1_ref[0]));
			amc_current[1] = -Kv*(state.dq1_1 - state.dq1_ref[1] + Kp*(state.q1_1 - state.q1_ref[1]) );
			if(!SKIP_AMC) 
			somatic_motor_cmd(&krang_cx.d_cx,&amc,SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,amc_current,2,NULL);
		}	break;
		// --------------------------------------------------------------------------------------------
		// To-Sit Mode:  Fix the wheel reference pos/vel and control imu angle to the reference 
		// sitting angle.
		case KRANG_MODE_TOSIT: {
			state.q1_ref[0] = state.q1_0;
			state.q1_ref[1] = state.q1_1;
			state.dq1_ref[0] = 0;
			state.dq1_ref[1] = 0;
			double K_TH	= 240;
			double K_DTH   = 60;
			amc_current[0] = K_TH*(state.q2 - SITTING_ANGLE) + K_DTH * state.dq2;
			amc_current[1] = amc_current[0];
			if(!SKIP_AMC) 
			somatic_motor_cmd(&krang_cx.d_cx,&amc,SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,amc_current,2,NULL);
		}	break;
		// --------------------------------------------------------------------------------------------
		// Balance Mode: Control imu angle to bring COM right above the wheel axisand control wheels
		// based on on joystick commands
		case KRANG_MODE_BALANCE: {
			static bool override=0;
			if(!override) { 
				printf("Setting the override mode!\n");
				somatic_motor_digital_out(&krang_cx.d_cx, &amc, 19 ,1);
				override=1; 
			}
			// Gains
			static const double KP_TH	= 745.0;
			static const double KD_TH   = 149.0;
			static const double KP_WH	 = 3.0;				
			static const double KD_WH	 = 10;				
			static const double KD_WH_LR = 15.0;
			// Errors
			static Dynamics dynamics (TOP_LEVEL_PATH"/data/MassProp.table", true);
			Eigen::VectorXd lq(7), rq(7);
			Eigen::Vector3d com = dynamics.com(state.q2, state.q3, 0.0, lq, rq);
			double error_th = atan2(com(0), com(1));// + 4*M_PI/180.0;
			double derror_th = state.dq2;
			double pref = (state.q1_ref[0] + state.q1_ref[1]) / 2.0;
			double error_wh = state.q1 - pref;
			double vref = (state.dq1_ref[0] + state.dq1_ref[1]) / 2.0;
			double derror_wh = state.dq1 - vref;
			// Control Law
			double u = ((KP_TH * error_th) + (KD_TH * derror_th)) + ((KD_WH *derror_wh) + (KP_WH * error_wh));
			double offset = KD_WH_LR * state.js_lr;
			amc_current[0] = u + offset;
			amc_current[1] = u - offset;
			if(!SKIP_AMC) 
			somatic_motor_cmd(&krang_cx.d_cx,&amc,SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,amc_current,2,NULL);
		}	break;
		// --------------------------------------------------------------------------------------------
		case KRANG_MODE_QUIT:
		case KRANG_MODE_BAD:
		case KRANG_MODE_SIZE:
		default: 
			somatic_sig_received = 1;
	}	// enbd of balancing switch
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
	ach_close(&js_chan);
	ach_close(&imu_chan);
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
