/**
 * @file krang.cpp
 * @author Kasemsit Teeyapan, Can Erdogan, Munzir Zafar, Kyle Volle
 * @date Aug 15, 2010
 * @brief The interface to talk with the main class that represents the state and the modules
 * of Krang. 
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include <amino.h>
#include <schkin.h>
#include "krang.h"
#include "krang-io.h"

/* ********************************************************************************************** */
Krang::Krang(krang_state_t* _state) {

	state = _state;

	// Initialize the kalman filter
	kf = new filter_kalman_t;
	filter_kalman_init( kf , 8 , 0 , 8 );

	// Create the interfaces for the joystick, imu, amc and schunk modules
	js	= new Joystick("joystick-data");
	imu   = new Imu("imu-data", 0.0);
	amc   = new Motor("amc-cmd", "amc-state", "AMC", 2);
	waist = new Motor("waist-cmd", "waist-state", "Waist", 2);

	// Set the minimum and maximum position of the waist modules
	aa_fset( waist->motor.pos_valid_min, -5, 2 );
	aa_fset( waist->motor.pos_valid_max, 5, 2 );

	// Reset PCIO
	waist->reset();

	// Check waist position (module id 14 and 15)
	if (fabs(waist->get_pos(0) + waist->get_pos(1)) > 2e-2)
		fprintf(stderr, "ERROR: Waist position mismatched: (%f, %f)\n", waist->get_pos(0), 
			-waist->get_pos(1));
}


/* ********************************************************************************************** */
Krang::~Krang() {

	// Halt the modules
	waist->halt();
	delete js;
	delete imu;
	delete amc;
	delete waist;
	filter_kalman_destroy( kf );
	delete( kf );
}

/* ********************************************************************************************** */
void Krang::get_js(krang_js_t *ejs) {
	memcpy(ejs, &this->js->jsvals, sizeof(*ejs));
}

/* ********************************************************************************************** */
void Krang::update() {

	// Update all the devices connected
	js->update();
	imu->update();
	amc->update();
	waist->update();
}

/* ********************************************************************************************** */
void Krang::update_and_filter() {

	// Look for updates on all the channels such as wheels, arms, joystick and etc.
	this->update();

	static bool debug = 0;
	static int counter = 0;

	// Set up the kalman filter with the raw data for imu, waist and amc
	// NOTE: The arm values are not filtered. Why not?
	kf->z[0] = imu->get_th();  // = q2
	kf->z[1] = imu->get_dth(); // = dq2
	kf->z[2] = amc->get_pos(0)+imu->get_th();  // = abs. L wheel pos.
	kf->z[3] = amc->get_vel(0)+imu->get_dth(); // = abs. L wheel vel.
	kf->z[4] = amc->get_pos(1)+imu->get_th();  // = abs. R wheel pos.
	kf->z[5] = amc->get_vel(1)+imu->get_dth(); // = abs. R wheel vel.
	kf->z[6] = waist->get_pos(0);
	kf->z[7] = waist->get_vel(0);

	// Call the kalman filter
	krang_kalman_filter();

	// Set the filtered values to the local fields
	state->q2		  = kf->x[0];
	state->dq2		 = kf->x[1];
	state->q1_0		= kf->x[2] - state->q2;
	state->dq1_0	   = kf->x[3] - state->dq2;
	state->q1_1		= kf->x[4] - state->q2;
	state->dq1_1	   = kf->x[5] - state->dq2;
	state->q3		  = kf->x[6];
	state->dq3		 = kf->x[7];

	// Useful debugging information to calibrate the kalman filter constants
	if(debug) {
		printf("%lf\t%lf\t%lf\t%lf\n", imu->get_th(), imu->get_dth(), kf->x[0], kf->x[1]);
		counter++;
		if(counter % 500 == 1) {
		  // printf("counter: %lu\n", counter);
			fflush(stdout);
		}
	}

	// Set the wheel position by taking the mean of the two wheel encoders 
	state->q1 = (state->q1_0 + state->q1_1)/2.0;
	state->dq1 = (state->dq1_0 + state->dq1_1)/2.0;
}

/* ********************************************************************************************** */
void Krang::krang_kalman_filter(/*double* state, double *raw_data*/) {

	// For now, assume fixed.
	double T = 2.04 * 1e-3;  // second

	static bool init_kf1 = false;
	if (!init_kf1) {
		memcpy(kf->x, kf->z, sizeof(double)*8);
		init_kf1 = true;
	}

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

	filter_kalman_predict( kf );
	filter_kalman_correct( kf );
}

