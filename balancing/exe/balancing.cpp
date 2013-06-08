#include <somatic.h>
#include <somatic/daemon.h>
#include <amino.h>
#include <Eigen/Dense>
#include <iostream>
#include <unistd.h>
#include <string>
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
#include <dirent.h>
#include "balancing.h"
#include "Dynamics.h"


/* ********************************************************************************************* */
// Global variables
krang_cx_t krang_cx;									
krang_state_t state;
somatic_motor_t amc, waist; 
ach_channel_t imu_chan, js_chan;
filter_kalman_t *kf;
DIR * dataFolder;
struct dirent *dataFolderEntries;
FILE * dumpFile;
double t_elapsed, t_elapsed_using_dt, t_elapsed_using_dt_js, t_sine;
double error_th, derror_th;
double dq1_ref=0.0;	
#define RAD2DEG(x) (x*180.0/M_PI)

// Gains
double Kp_sit = 0;
double Kv_sit = 20;
double K_TH_toSit	= 240;
double K_DTH_toSit   = 60;
static const double KP_TH	= 309.0833;
static const double KD_TH   = 38.6935;
static const double KP_WH	 = 10;				
static const double KD_WH	 = 15.8498;				
static const double KD_WH_LR = 15.0;
/* ********************************************************************************************* */
// Initialize the program
void init() {
	somatic_d_opts_t dopt;
	memset(&krang_cx, 0, sizeof(krang_cx)); // zero initialize
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "bal-hack";
	somatic_d_init( &(krang_cx.d_cx), &dopt );

	// Set initial mode
	state.mode = KRANG_MODE_HALT;

	// Initialize the motors with daemon context, channel names and # of motors
	somatic_motor_init( &krang_cx.d_cx, &amc, 2, "amc-cmd", "amc-state");

	// Set the min and maximum position and velocity valid/limit values for motors
	double ** limits[] = { 
		&amc.pos_valid_min, &amc.vel_valid_min, &amc.pos_limit_min, &amc.vel_limit_min, 
	  &amc.pos_valid_max,	&amc.vel_valid_max, &amc.pos_limit_max, &amc.vel_limit_max};
	for(size_t i=0; i<4; i++) { aa_fset(*limits[i],-1024.1, 2); }
	for(size_t i=4; i<8; i++) { aa_fset(*limits[i],1024.1, 2); }
	usleep(1e5);

	// Update and reset motors
	somatic_motor_update(&krang_cx.d_cx, &amc);

	// Open the IMU channel
	int r  = ach_open( &imu_chan, "imu-data" , NULL );
	aa_hard_assert(r == ACH_OK, "Ach failure %s on opening IMU channel (%s, line %d)\n",
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open Joystick channel
	r  = ach_open(&js_chan, "joystick-data" , NULL);
	aa_hard_assert(r == ACH_OK,
				   "Ach failure %s on opening Joystick channel (%s, line %d)\n",
				   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open the state channel
	somatic_d_channel_open( &(krang_cx.d_cx),
							&(krang_cx.state_chan), "krang-state",
							NULL );
	
	// Initialize the kalman filter
	kf = new filter_kalman_t;
	filter_kalman_init( kf , 8 , 0 , 8 );

	// Create file for dumping data
	int dumpFileNumber=0; char dumpFilePrefix[]="balance-dump-";
	// Open data folder and browse through existing dump files to pick the largest file number
	if ((dataFolder = opendir (TOP_LEVEL_PATH"/data")) != NULL) {
		while ((dataFolderEntries = readdir (dataFolder)) != NULL) {
			if(strncmp(dataFolderEntries->d_name, dumpFilePrefix, strlen(dumpFilePrefix))==0) {
				dumpFileNumber=std::max(dumpFileNumber, atoi(dataFolderEntries->d_name+strlen(dumpFilePrefix)));
			}
		}
		// Close the data folder
		closedir (dataFolder);
		// Create new dump file
		dumpFileNumber++;
		char dumpFileName[strlen(TOP_LEVEL_PATH) + 6 + strlen(dumpFilePrefix) + 9];
		sprintf(dumpFileName, TOP_LEVEL_PATH"/data/%s%05d.dat", dumpFilePrefix, dumpFileNumber);
		dumpFile = fopen(dumpFileName, "w");
	} 
	// Failure opening data folder
	else {
		printf("Error Opening data folder\n");
	}	

	// Dump a comment describing the experiment
	fprintf(dumpFile, "# This file contains the sensor readings, joystick readings, and wheel ");
	fprintf(dumpFile, "motor current (control input) alon with time stamps for ease of plotting\n");
	fprintf(dumpFile, "# Sit mode gains: Kp_sit=%lf, Kv_sit=%lf\n", Kp_sit, Kv_sit);
	fprintf(dumpFile, "# To-Sit mode gains: K_TH_toSit=%lf, K_DTH_toSit=%lf\n", K_TH_toSit, K_DTH_toSit);
	fprintf(dumpFile, "# Balance mode gains: KP_TH=%lf, KD_TH=%lf, KP_WH=%lf, KD_WH=%lf, KP_LR=%lf\n",
		KP_TH, KD_TH, KP_WH, KD_WH, KD_WH_LR);
	fprintf(dumpFile, "# 1.time 2.error_th 3.rawImuPos 4.rawImuVel 5.rawLWhPos 6.rawLWhVel "
		"7.rawRWhPos 8.rawRWhVel 9.rawWaistPos 10.rawWaistVel	11.filtImuPos 12.filtImuVel "
		"13.filtLWhPos 14.filtLWhVel 15.filtRWhPos 16.filtRWhVel 17.filtWaistPos 18.filtWaistVel "
		"19.avgWhPos 20.avgWhVel 21.jsFB 22.jsLR 23.refLWhPos 24.refLWhVel 25.refRWhPos 26.refRWhVel"
		"27.avgRefWhPos 28.avgRefWhVel 29.cmdLWhCur 30.cmdRWhCur 31.rawLWhCur 32.rawRWhCur 33.mode\n");
}

/* ********************************************************************************************* */
// Read sensor data from ach channels and fulter out the noise
void readSensors(double dt) {
	
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
		kf->z[0] = ssdmu_pitch(&imu_sample);				 
		kf->z[1] = ssdmu_d_pitch(&imu_sample);			

		// Free the unpacked message
		somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );
	}

	// Read wheel position and speed
	somatic_motor_update(&krang_cx.d_cx, &amc);
	kf->z[2] = amc.pos[0]+kf->z[0];  // = abs. L wheel pos.
	kf->z[3] = amc.vel[0]+kf->z[1]; // = abs. L wheel vel.
	kf->z[4] = amc.pos[1]+kf->z[0];  // = abs. R wheel pos.
	kf->z[5] = amc.vel[1]+kf->z[1]; // = abs. R wheel vel.

	// Read waist position and speed
	kf->z[6] = 117*M_PI/180.0;;
	kf->z[7] = 0.0;
	
	// If first reading, set motor offsets
	static bool offsetDone=0;
	if(offsetDone==0) {
		double amc_pos_offset[2] = {-kf->z[2], -kf->z[4]};
		aa_fcpy(amc.pos_offset, amc_pos_offset, 2);

		kf->z[2] = 0.0;
		kf->z[4] = 0.0;
		kf->z[6] = 117*M_PI/180.0;
		offsetDone = 1;
		return;
	}
	
	// --------------------------------------------------------------------
	// Filter the read values	
	static bool firstIteration=1;
	if(firstIteration) { memcpy(kf->x, kf->z, sizeof(double)*8); firstIteration=0;}

	double T = dt; // second
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


	// Filter the noise	
	filter_kalman_predict( kf );
	filter_kalman_correct( kf );

	// Save the output of the filter in the state structure
	state.q2		  = kf->x[0];
	state.dq2		 = kf->x[1];
	state.q1_0		= kf->x[2] - state.q2;
	state.dq1_0	   = kf->x[3] - state.dq2;
	state.q1_1		= kf->x[4] - state.q2;
	state.dq1_1	   = kf->x[5] - state.dq2;
	state.q3		  = 117*M_PI/180.0; //FIXME: This is a hack and works for a fixed waist angle. It should have been kf->x[6];
																	// But for some reason it is behaving strange. Value is slowwwly rising to 117 from 0.
	state.dq3		 = 0.0;//kf->x[7];

	// Set the wheel position by taking the mean of the two wheel encoders 
	state.q1 = (state.q1_0 + state.q1_1)/2.0;
	state.dq1 = (state.dq1_0 + state.dq1_1)/2.0;
	// --------------------------------------------------------------------
	// If IMU angle is below the sitting angle and if current mode is TOSIT, change mode toSIT
	double imu = state.q2, epsilon=.001;
	if(imu < (SITTING_ANGLE - epsilon)) { if(state.mode == KRANG_MODE_TOSIT) state.mode = KRANG_MODE_SIT; }

	// Compute center of mass
	static Dynamics dynamics (TOP_LEVEL_PATH"/data/MassProp.table", true);
	Eigen::VectorXd lq(7), rq(7);
	Eigen::Vector3d com = dynamics.com(state.q2, state.q3, 0.0, lq, rq);

	// Compute the error terms
	// making it zero for an experiment
	error_th = atan2(com(0), com(1));// + 4*M_PI/180.0;
	derror_th = state.dq2;
	if(error_th < -STABLE_TH_RANGE ) { 
		if(state.mode == KRANG_MODE_BALANCE || state.mode == KRANG_MODE_TRACK_SINE)	
			state.mode = KRANG_MODE_TOSIT;
	}
	if(error_th > STABLE_TH_RANGE ) {
		if(state.mode == KRANG_MODE_BALANCE || state.mode == KRANG_MODE_TRACK_SINE)	
			state.mode = KRANG_MODE_HALT;
	}
}

/* ********************************************************************************************* */
// Read and process the commands from the joystick
void readJoystick( double dt ) {
	
	// Get the values
	static char b [10], b_prev[10];
	static double x [6];

	// PRevious button states
	for(size_t i = 0; i < 10; i++) 
		b_prev[i] = b[i];

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if((ACH_OK == r || ACH_MISSED_FRAME == r) && js_msg != NULL )
	{
		for(size_t i = 0; i < 10; i++) {
			b[i] = js_msg->buttons->data[i] ? 1 : 0;
		}
		memcpy(x, js_msg->axes->data, sizeof(x));
	}	
	// Set the forward/backward ad left/right joystick axes values to zero
	state.js_fb = 0;	state.js_lr = 0;

	t_elapsed_using_dt_js += dt;

	// Check for the start, quit, sit and stand conditions
	if(b[6] && b[7]) { if(state.mode == KRANG_MODE_HALT) { state.mode=KRANG_MODE_SIT; printf("MODE_SIT\n"); dq1_ref=0.0; }}
	else if(b[8]) { state.mode = KRANG_MODE_QUIT; }
	//	else if(b[9] && !b[5]) { if(state.mode == KRANG_MODE_BALANCE) state.mode = KRANG_MODE_TOSIT; } 
	else if(b[9] && b[5]) { if(state.mode == KRANG_MODE_SIT || state.mode == KRANG_MODE_TRACK_SINE) { state.mode = KRANG_MODE_BALANCE; dq1_ref=0.0; } }
	else if(b[5] && b[0]) { if(state.mode == KRANG_MODE_BALANCE) { state.mode = KRANG_MODE_TRACK_SINE; t_sine=0.0; } }
	// If button 4 is pressed (indexed 3) in balance-mode increase dq1_ref by 0.01
	else if(b_prev[3] && !b[3]) { if(state.mode == KRANG_MODE_BALANCE) dq1_ref+=0.01; printf("dq1_ref=%lf\n",dq1_ref); }
	// If button 2 is pressed (indexed 1) in balance-mode decrease dq1_ref by 0.01
	else if(b_prev[1] && !b[1]) { if(state.mode == KRANG_MODE_BALANCE) dq1_ref-=0.01; printf("dq1_ref=%lf\n",dq1_ref); }
	// If button 3 is pressed (indexed 2) in balance-mode make q1_ref=0
	else if(b_prev[2] && !b[2]) { if(state.mode == KRANG_MODE_BALANCE) dq1_ref=0.0; printf("dq1_ref=%lf\n",dq1_ref); }
	// Check for the wheel control - no shoulder button should be pressed
	// None of the shoulder buttons should be pressed
	else if(!b[4] && !b[5] && !b[6] && !b[7]) {
		state.js_fb = -x[1];	// range [-1, 1]
		state.js_lr =  x[2];	// range [-1, 1]
		
		// Generate velocity reference based on the mode
		// If we are not in the TRACK_SINE mode, use the joystick to generate velocity reference
		if( state.mode != KRANG_MODE_TRACK_SINE )	{
			// Velocity control when sitting
			state.dq1_ref[0] = MAX_LIN_VEL*state.js_fb;
			state.dq1_ref[1] = state.dq1_ref[0];

			// Moving left/right: Velocity control for heading
			double max_ang_vel = 2.0;
			state.dq1_ref[0] += max_ang_vel*state.js_lr;
			state.dq1_ref[1] -= max_ang_vel*state.js_lr;
			/*state.dq1_ref[0] = dq1_ref;
			state.dq1_ref[1] = dq1_ref;*/
		}
		// If in TRACK_SINE mode ignore joystick and generate a sinusoidal velocity reference
		else {
			double freq=1/12.0; double Amplitude=2.0; // One cycle in six seconds
			state.dq1_ref[0] = Amplitude*2*M_PI*freq*cos(2*M_PI*freq*t_sine);
			state.dq1_ref[1] = Amplitude*2*M_PI*freq*cos(2*M_PI*freq*t_sine);
			t_sine += dt;
		}
		// integrate
		state.q1_ref[0] += dt * state.dq1_ref[0];
		state.q1_ref[1] += dt * state.dq1_ref[1];

		//printf("t_elapsed=%lf, t_elapsed_using_dt=%lf, t_elapsed_using_dt_js=%lf dq1_ref=%lf, q1_ref=%lf\n", 
		//	t_elapsed, t_elapsed_using_dt, t_elapsed_using_dt_js, state.dq1_ref[0], state.q1_ref[0] );

		// avg ref wheel pos/vel for forward/backward control
		state.pref = (state.q1_ref[0] + state.q1_ref[1]) / 2.0;
		state.vref = (state.dq1_ref[0] + state.dq1_ref[1]) / 2.0;
	} 
}

/* ********************************************************************************************* */
// Control the wheels based on the current mode
void controlWheels(double dt) {
	bool SKIP_AMC=0;
	switch(state.mode) {
		// --------------------------------------------------------------------------------------------
		// Halt Mode: Fix the wheel reference pos/vel to current pos/vel
		case KRANG_MODE_HALT: {
			state.q1_ref[0] = state.q1_0;
			state.q1_ref[1] = state.q1_1;
			state.dq1_ref[0] = 0;
			state.dq1_ref[1] = 0;
			state.u[0]=0; state.u[1]=0;
		}	break;
		// --------------------------------------------------------------------------------------------
		// Sit Mode: Only control the wheel pos/vel (and not imu pos/vel)
		case KRANG_MODE_SIT: {
			state.u[0] = -Kv_sit*(state.dq1_0 - state.dq1_ref[0] + Kp_sit*(state.q1_0 - state.q1_ref[0]));
			state.u[1] = -Kv_sit*(state.dq1_1 - state.dq1_ref[1] + Kp_sit*(state.q1_1 - state.q1_ref[1]) );
			bool debug=1; static int debug_cnt=0;
			if(debug & debug_cnt++ % 500 == 0) { 
				printf("\nimu: %.3lf, waist: %.3lf, error_th: %.3lf\n", state.q2*180.0/M_PI, state.q3*180.0/M_PI, error_th*180.0/M_PI);
				printf("amc_current = { %.3lf, %.3lf }  ", state.u[0], state.u[1]);
				printf("dt=%.5lf\n", dt);
			}
		}	break;
		// --------------------------------------------------------------------------------------------
		// To-Sit Mode:  Fix the wheel reference pos/vel and control imu angle to the reference 
		// sitting angle.
		case KRANG_MODE_TOSIT: {
			state.q1_ref[0] = state.q1_0;
			state.q1_ref[1] = state.q1_1;
			state.dq1_ref[0] = 0;
			state.dq1_ref[1] = 0;
			state.u[0] = K_TH_toSit*(state.q2 - SITTING_ANGLE) + K_DTH_toSit * state.dq2;
			state.u[1] = state.u[0];
		}	break;
		// --------------------------------------------------------------------------------------------
		// Balance Mode: Control imu angle to bring COM right above the wheel axisand control wheels
		// based on on joystick commands
		case KRANG_MODE_BALANCE:
		case KRANG_MODE_TRACK_SINE: {
			static bool override=0;
			if(!override) { 
				printf("Setting the override mode!\n");
				somatic_motor_digital_out(&krang_cx.d_cx, &amc, 19 ,1);
				override=1; 
			}
			// Gains

			double error_wh = state.q1 - state.pref;
			double derror_wh = state.dq1 - state.vref;

			// Control Law
			double u = ((KP_TH * error_th) + (KD_TH * derror_th)) + ((KD_WH *derror_wh) + (KP_WH * error_wh));
			double offset = KD_WH_LR * state.js_lr;
			state.u[0] = u + offset;
			state.u[1] = u - offset;
			bool debug=1; static int debug_cnt=0;
			if(debug & debug_cnt++ % 500 == 0) { 
				printf("\nimu: %.3lf, waist: %.3lf, error_th: %.3lf \n ", state.q2*180.0/M_PI, state.q3*180.0/M_PI, error_th*180.0/M_PI);
				printf("amc_current = { %.3lf, %.3lf }  ", state.u[0], state.u[1]);
				printf("dt=%.5lf\n", dt);
			}
		}	break;
		// --------------------------------------------------------------------------------------------
		case KRANG_MODE_QUIT:
		case KRANG_MODE_BAD:
		case KRANG_MODE_SIZE:
		default: 
			state.u[0] = 0; state.u[1]=0;
			somatic_sig_received = 1;
	}	// enbd of balancing switch
	if(!SKIP_AMC) 
		somatic_motor_cmd(&krang_cx.d_cx,&amc,SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,state.u,2,NULL);
}

/* ********************************************************************************************* */
// Dump all the current states to file
inline void dumpToLog( char * log, int index ) {
	sprintf(log+index, "%011.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf "	
		"%09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf "	
		"%06.3lf %06.3lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %09.5lf %07.3lf %07.3lf %07.3lf "
		"%07.3lf %d\n", 			
		t_elapsed, error_th, kf->z[0], kf->z[1], amc.pos[0], amc.vel[0], amc.pos[1], amc.vel[1], 
		kf->z[6],	kf->z[7],	state.q2, state.dq2, state.q1_0, state.dq1_0, state.q1_1, state.dq1_1, 
		state.q3,	state.dq3, state.q1, state.dq1, state.js_fb, state.js_lr, state.q1_ref[0], 
		state.dq1_ref[0],	state.q1_ref[1], state.dq1_ref[1], state.pref, state.vref, state.u[0], 
		state.u[1], amc.cur[0],	amc.cur[1], state.mode);
}
/* ********************************************************************************************* */
// This is the main loop that interfaces with the I/O from the joystick.
void run() {
	// Send the event massage
	somatic_d_event( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_RUNNING,
					 NULL, NULL );
	static char * log= new char[50000000];
	static int log_index=0;
  static int log_interleave_count=0;
	// Set the timestep to update
	double dt = 0.0;
	t_elapsed_using_dt=0.0; t_elapsed_using_dt_js=0.0;
	// Get the time in the beginning
	struct timespec t_now, t_prev, t_start;
	t_start = aa_tm_now();
	t_prev = t_start;
	while (!somatic_sig_received ) {
		// Get the current time and compute the time difference
		t_now = aa_tm_now();						
		dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_elapsed = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_start));
		t_elapsed_using_dt += dt;
		// Get wheel, imu, waist positions and velocities from ack chennels
		readSensors(dt);
		// Get commands from the joystick and process them
		readJoystick(dt);
		// Based on the sensor feedback, joystick commands and current state, call the relevant 
		// controller for controlling the wheels
		controlWheels(dt);
		// Dump data to file after skipping 10 samples
	  if(++log_interleave_count>10){ 
			// Write to the log string
			dumpToLog(log, log_index);
			// Increment the index
			log_index+=310;
			// If the log string overflows, wrap around
			if(log_index>50000000){ log_index=0; }
			log_interleave_count=0;
		}
		// Update previous measured time
		t_prev = t_now;
		// Release memory
		aa_mem_region_release( &krang_cx.d_cx.memreg );
	}
	// Write the log to the dumpfile
	fprintf(dumpFile,"%s",log);
	// Once the main loop returns, the program is done. Send the stopping event message.
	somatic_d_event( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING,
					 NULL, NULL );
}

/* ********************************************************************************************* */
void destroy() {
	ach_close(&js_chan);
	ach_close(&imu_chan);
	somatic_motor_digital_out(&krang_cx.d_cx, &amc, 19, 0);
//	somatic_motor_cmd(&krang_cx.d_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 2, NULL);
	state.u[0]=0.0; state.u[1]=0.0;
	somatic_motor_cmd(&krang_cx.d_cx,&amc,SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,state.u,2,NULL);
	somatic_motor_destroy(&krang_cx.d_cx, &amc);
	filter_kalman_destroy( kf );
	delete( kf );
	somatic_d_destroy( &krang_cx.d_cx );
	fclose(dumpFile);
}

/* ********************************************************************************************* */
int main() {
	init();
	run();
	destroy();
	return 0;
}
