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
using namespace std;

/* ********************************************************************************************* */
// Global variables
krang_cx_t krang_cx;									
krang_state_t state;
somatic_motor_t amc, waist; 
ach_channel_t imu_chan, js_chan, waistd_chan;
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
static double KP_TH	= 300.0833;
static double KD_TH   = 38.6935;
static double KP_WH	 = 0.0;//17.5;				
static double KD_WH	 = 0.0;				
static double KP_WH_LR = 15.0;
static double KD_WH_LR = 15.0;

// Parameters
double wheelRadius = 10.5; // inches
double distanceBetweenWheels = 27.375; // inches

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
	somatic_motor_init( &krang_cx.d_cx, &waist, 2, "waist-cmd", "waist-state" );

	// Set the min and maximum position and velocity valid/limit values for motors
	double ** limits[] = { 
		&amc.pos_valid_min, &amc.vel_valid_min, &amc.pos_limit_min, &amc.vel_limit_min, 
	  &waist.pos_valid_min, &waist.vel_valid_min, &waist.pos_limit_min, &waist.vel_limit_min,	
		&amc.pos_valid_max,	&amc.vel_valid_max, &amc.pos_limit_max, &amc.vel_limit_max,	
		&waist.pos_valid_max, &waist.vel_valid_max, &waist.pos_limit_max, &waist.vel_limit_max};
	for(size_t i=0; i<8; i++)  { aa_fset(*limits[i],-1024.1, 2); }
	for(size_t i=8; i<16; i++) { aa_fset(*limits[i],1024.1, 2); }
	aa_fset( waist.pos_valid_min, -5, 2); 
	aa_fset( waist.pos_valid_max, 5, 2); 
	
	// Update and reset motors
	somatic_motor_update(&krang_cx.d_cx, &amc);
	somatic_motor_update(&krang_cx.d_cx, &waist);
	double vals[] = {0.0,0.0};
  somatic_motor_cmd(&krang_cx.d_cx, &waist, SOMATIC__MOTOR_PARAM__MOTOR_RESET, vals, 2, NULL);
	usleep(1e5);

	// Open the IMU channel
	int r  = ach_open( &imu_chan, "imu-data" , NULL );
	aa_hard_assert(r == ACH_OK, "Ach failure %s on opening IMU channel (%s, line %d)\n",
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open Joystick channel
	r  = ach_open(&js_chan, "joystick-data" , NULL);
	aa_hard_assert(r == ACH_OK,
				   "Ach failure %s on opening Joystick channel (%s, line %d)\n",
				   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open waist daemon command channel
	r  = ach_open(&waistd_chan, "waistd-cmd" , NULL);
	aa_hard_assert(r == ACH_OK,
				   "Ach failure %s on opening Waist Daemon Command channel (%s, line %d)\n",
				   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open the state channel
	somatic_d_channel_open( &(krang_cx.d_cx),
							&(krang_cx.state_chan), "krang-state",
							NULL );
	
	// Initialize the kalman filter
	kf = new filter_kalman_t;
	filter_kalman_init( kf , 8 , 0 , 8 );

	// Create file for dumping data
	int dumpFileNumber=0; char dumpFilePrefix[]="tuning-dump-";
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
	fprintf(dumpFile, "# Balance mode gains: KP_TH=%lf, KD_TH=%lf, KP_WH=%lf, KD_WH=%lf, KP_LR=%lf, "
		"KD_LR=%lf\n", KP_TH, KD_TH, KP_WH, KD_WH, KP_WH_LR, KD_WH_LR);
	fprintf(dumpFile, "# 1.time 2.error_th 3.rawImuPos 4.rawImuVel 5.rawLWhPos 6.rawLWhVel "
		"7.rawRWhPos 8.rawRWhVel 9.rawWaistPos 10.rawWaistVel	11.filtImuPos 12.filtImuVel "
		"13.filtLWhPos 14.filtLWhVel 15.filtRWhPos 16.filtRWhVel 17.filtWaistPos 18.filtWaistVel "
		"19.avgWhPos 20.avgWhVel 21.jsFB 22.jsLR 23.refLWhPos 24.refLWhVel 25.refRWhPos 26.refRWhVel"
		"27.avgRefWhPos 28.avgRefWhVel 29.cmdLWhCur 30.cmdRWhCur 31.rawLWhCur 32.rawRWhCur 33.mode "
		"34.spin 35.spinSpeed 36.spinRef 37.spinSpeedRef 38.X 39.Y 40.KP_TH 41.KD_TH 42.KP_WH"
		"43.KD_WH 44.KP_LR 45.KD_LR\n");

	state.x=0.0;
	state.y=0.0;
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
	static bool waistErrorPrinted=0;
	somatic_motor_update(&krang_cx.d_cx, &waist);
	if(waist.pos[0]+waist.pos[1]>0.087 && !waistErrorPrinted) {
		printf("ERROR: Waist modules are sensed to be misaligned!\n"); 
		printf("[%lf, %lf]", waist.pos[0], waist.pos[1]);
		waistErrorPrinted=1;
	}
	kf->z[6] = (waist.pos[0]-waist.pos[1])/2;
	kf->z[7] = (waist.vel[0]-waist.vel[1])/2;;
	
	// If first reading, set motor offsets
	static bool offsetDone=0;
	if(offsetDone==0) {
		double amc_pos_offset[2] = {-kf->z[2], -kf->z[4]};
		aa_fcpy(amc.pos_offset, amc_pos_offset, 2);

		kf->z[2] = 0.0;
		kf->z[4] = 0.0;
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
	state.q3		  = kf->x[6];
	state.dq3		 = kf->x[7];

	// Set the wheel position by taking the mean of the two wheel encoders 
	state.q1 = (state.q1_0 + state.q1_1)/2.0;
	state.dq1 = (state.dq1_0 + state.dq1_1)/2.0;
	
	// Determine the spin and spin rate of the robot
	state.spin = wheelRadius * (state.q1_1 - state.q1_0) / distanceBetweenWheels;
	state.dspin = wheelRadius * (state.dq1_1 - state.dq1_0) / distanceBetweenWheels;

	// Determine the x and y values of the robot by integrating the x and y components of the heading speed
	// x-axis is assumed to be the initial position of wheel axis from left to right
	// y-axis is assumed to be the initial forward direction
	state.x+=-wheelRadius*0.0254*state.dq1*sin(state.spin)*dt;
	state.y+= wheelRadius*0.0254*state.dq1*cos(state.spin)*dt;
	
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
		if(state.mode == KRANG_MODE_BALANCE || state.mode == KRANG_MODE_TRACK)	
			state.mode = KRANG_MODE_TOSIT;
	}
	if(error_th > STABLE_TH_RANGE ) {
		if(state.mode == KRANG_MODE_BALANCE || state.mode == KRANG_MODE_TRACK)	
			state.mode = KRANG_MODE_HALT;
	}
}

/************************************************************************************************/
// Control the waist motors based on joystick input. This is done by communicating to the 
// krang-waist daemon via the waistd-cmd ach channel.

void waistCtrl( const double jsInput ) { 
	// Decide what mode to choose for the waist motor control
	static Somatic__WaistMode waistMode;
	if(jsInput < -0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
	else if(jsInput > 0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
	else waistMode = SOMATIC__WAIST_MODE__STOP;

	// Send message to the krang-waist daemon
	static Somatic__WaistCmd *waistDaemonCmd=somatic_waist_cmd_alloc();
	somatic_waist_cmd_set(waistDaemonCmd, waistMode);
	somatic_metadata_set_time_now(waistDaemonCmd->meta);
	somatic_metadata_set_until_duration(waistDaemonCmd->meta, .1);
	int r = SOMATIC_PACK_SEND( &waistd_chan, somatic__waist_cmd, waistDaemonCmd );
	if( ACH_OK != r ) {
			fprintf(stderr, "Couldn't send message: %s\n",
							ach_result_to_string(static_cast<ach_status_t>(r)));
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
			memcpy(x, js_msg->axes->data, sizeof(x)); 
			for(size_t i = 0; i < 10; i++) {
				b[i] = js_msg->buttons->data[i] ? 1 : 0;
			}

	}
	// Waist Control based on joystick data
	waistCtrl( x[5] );

	// Set the forward/backward ad left/right joystick axes values to zero
	state.js_fb = 0;	state.js_lr = 0;

	t_elapsed_using_dt_js += dt;

	// Check for the start, quit, sit and stand conditions
	if(b[6] && b[7]) { if(state.mode == KRANG_MODE_HALT) { state.mode=KRANG_MODE_SIT; printf("MODE_SIT\n"); dq1_ref=0.0; }}
	else if(b[8]) { state.mode = KRANG_MODE_QUIT; }
	//	else if(b[9] && !b[5]) { if(state.mode == KRANG_MODE_BALANCE) state.mode = KRANG_MODE_TOSIT; } 
	else if(b[9] && b[5]) { if(state.mode == KRANG_MODE_SIT || state.mode == KRANG_MODE_TRACK) { state.mode = KRANG_MODE_BALANCE; dq1_ref=0.0; } }
	else if(b[5] && (b[0] || b[1] || b[2])) { if(state.mode == KRANG_MODE_BALANCE) { 
			state.q1_ref[0]=state.q1_0;
			state.q1_ref[1]=state.q1_1;
			state.spin_ref = wheelRadius * (state.q1_ref[1]-state.q1_ref[0]) / distanceBetweenWheels;
			state.pref = (state.q1_ref[0] + state.q1_ref[1]) / 2.0;
			state.mode = KRANG_MODE_TRACK;
			t_sine=0.0; 
			if(b[0]) state.track_mode=TRACK_FWD_REV; 
			else if(b[1]) state.track_mode=TRACK_LEFT_RIGHT;
			else if(b[2]) state.track_mode=TRACK_CIRCLE;
		} 
	}
	// If button 5 or 7 are presses-and-released while buttons 1, 2, 3 or 4 were pressed change gains
	else if(((b_prev[4] && !b[4]) || (b_prev[6] && !b[6])) && (b[0]||b[1]||b[2]||b[3])) {
		double sign;
		// If button 5 is pressed increment values
		if(b_prev[4] && !b[4]) { sign=1.0; }
		// IF button 7 is pressed decrement values
		else if(b_prev[6] && !b[6]) { sign=-1.0; }
		
		// button 1, 2, 3 and 4 correspond to KP_TH, KD_TH, KP_WH and KD_WH respectively
		if(b[0]) KP_TH = max(100.0, min(500.0, KP_TH+sign*3.0));
		else if(b[1]) KD_TH = max(10.0, min(150.0, KD_TH+sign*1.0));
		else if(b[2]) KP_WH = max(0.0, min(50.0, KP_WH+sign*(1.0/3.0)));
		else if(b[3]) KD_WH = max(0.0, min(50.0, KD_WH+sign*(1.0/3.0)));
	}
	// Check for the wheel control - no shoulder button should be pressed
	// None of the shoulder buttons should be pressed
	else if(!b[4] && !b[5] && !b[6] && !b[7]) {
		state.js_fb = -x[1]*0.75;	// range [-1, 1]
		state.js_lr =  x[2]*0.5;	// range [-1, 1]
		
		// Generate velocity reference based on the mode
		// If we are not in the TRACK mode, use the joystick to generate velocity reference
		if( state.mode != KRANG_MODE_TRACK )	{
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
		// If in TRACK mode ignore joystick and generate a reference of the trajectory to be followed
		else {
			switch(state.track_mode) {
				case TRACK_FWD_REV: {
					double freq=1/12.0; double Amplitude=2.0; // One cycle in six seconds
					state.dspin_ref = 0.0;
					state.dq1_ref[0] = Amplitude*2*M_PI*freq*cos(2*M_PI*freq*t_sine);
					state.dq1_ref[1] = Amplitude*2*M_PI*freq*cos(2*M_PI*freq*t_sine);
					break;
				}
				case TRACK_LEFT_RIGHT: {
					double freq=1/12.0; double Amplitude=M_PI/2; // One cycle in six seconds
					state.dspin_ref= Amplitude*2*M_PI*freq*cos(2*M_PI*freq*t_sine);
					state.dq1_ref[0] = -distanceBetweenWheels*state.dspin_ref/(2*wheelRadius);
					state.dq1_ref[1] =  distanceBetweenWheels*state.dspin_ref/(2*wheelRadius);
					break;
				}
				case TRACK_CIRCLE: {
					double circleRadius = 30.0; // inches 
					double timePerRound = 10.0; // seconds
					state.dspin_ref=-2*M_PI/timePerRound;
					state.dq1_ref[0]=M_PI*(2*circleRadius+distanceBetweenWheels)/(wheelRadius*timePerRound);
					state.dq1_ref[1]=M_PI*(2*circleRadius-distanceBetweenWheels)/(wheelRadius*timePerRound);
				break;	
				}
			}
			t_sine += dt;
		}
		// The following line should only be used for left/right and circle trajectory
		state.spin_ref = wheelRadius * (state.q1_ref[1]-state.q1_ref[0]) / distanceBetweenWheels;
		
		// integrate
		state.q1_ref[0] += dt * state.dq1_ref[0];
		state.q1_ref[1] += dt * state.dq1_ref[1];

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
				printf("KP_TH: %05.1lf, KD_TH: %04.1lf, KP_WH: %04.1lf, KD_WH: %04.1lf \n", 
					KP_TH, KD_TH, KP_WH, KD_WH );
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
		case KRANG_MODE_TRACK: {
			static bool override=0;
			if(!override) { 
				printf("Setting the override mode!\n");
				somatic_motor_digital_out(&krang_cx.d_cx, &amc, 19 ,1);
				override=1; 
			}
			double error_wh = state.q1 - state.pref;
			double derror_wh = state.dq1 - state.vref;

			// Control Law
			double offset;
			double u = ((KP_TH * error_th) + (KD_TH * derror_th)) + ((KD_WH *derror_wh) + (KP_WH * error_wh));
			if(state.mode == KRANG_MODE_BALANCE) { 
				offset = KD_WH_LR * state.js_lr;
			} else {
				offset = KP_WH_LR*(state.spin-state.spin_ref) + KD_WH_LR*(state.dspin-state.dspin_ref);
			}
			
			state.u[0] = u + offset;
			state.u[1] = u - offset;
			bool debug=1; static int debug_cnt=0;
			if(debug & debug_cnt++ % 500 == 0) { 
				printf("\nimu: %.3lf, waist: %.3lf, error_th: %.3lf \n ", state.q2*180.0/M_PI, state.q3*180.0/M_PI, error_th*180.0/M_PI);
				printf("amc_current = { %.3lf, %.3lf }  ", state.u[0], state.u[1]);
				printf("dt=%.5lf\n", dt);
				printf("KP_TH: %05.1lf, KD_TH: %04.1lf, KP_WH: %04.1lf, KD_WH: %04.1lf \n", 
					KP_TH, KD_TH, KP_WH, KD_WH );
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
		"%07.3lf %d %09.5lf %09.5lf %09.5lf %09.5lf %07.3lf %07.3lf %05.1lf %05.1lf %04.1lf %04.1lf "
		"%04.1lf %04.1lf\n", 			
		t_elapsed, error_th, kf->z[0], kf->z[1], amc.pos[0], amc.vel[0], amc.pos[1], amc.vel[1], 
		kf->z[6],	kf->z[7],	state.q2, state.dq2, state.q1_0, state.dq1_0, state.q1_1, state.dq1_1, 
		state.q3,	state.dq3, state.q1, state.dq1, state.js_fb, state.js_lr, state.q1_ref[0], 
		state.dq1_ref[0],	state.q1_ref[1], state.dq1_ref[1], state.pref, state.vref, state.u[0], 
		state.u[1], amc.cur[0],	amc.cur[1], state.mode, state.spin, state.dspin, state.spin_ref,
		state.dspin_ref, state.x, state.y, KP_TH, KD_TH, KP_WH, KD_WH, KP_WH_LR, KD_WH_LR);
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
			log_index+=398;
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