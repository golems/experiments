/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and filtering...
 */

#include "helpers.h"

/* ********************************************************************************************* *
/// Gets initial wheel positions to be subtracted from all subsequent readings. This is because
/// the initial values are to be treated as zeros.
void getWheelInitial(double* _wheelInitial[]) {
	
	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	
	// Reads amc position from the amc state channel
	Somatic__MotorState* amc; int r;
	assert(((amc = getMotorMessage(amcChan)) != NULL) && "Wheels call failed");

	// Get imu position
	double imu, imuSpeed;
	getImu(imu, imuSpeed);

	// Store in amcOffset the sum of amc-encoders and imu value. This is done to compensate for the
	// effect of imu rotation in encoder readings.
	*_wheelInitial[0] = amc->position->data[0]+imu;
	*_wheelInitial[1] = amc->position->data[1]+imu;
}

/* ********************************************************************************************* */
/// Filters the imu, wheel and waist readings 
void filterState (double dt, filter_kalman_t* kf, Eigen::VectorXd& q, Eigen::VectorXd& dq) {

	// Set the data of the filter (note the wheel and imu positions are added).
	for (int i = 0; i < 8; i++)  kf->z[i] = (i % 2) ? dq(5 + i/2) : q(5 + i/2);  
	
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
	for( int i = 5, j = 0; i < 9; i++) { q(i) = kf->x[j++]; dq(i) = kf->x[j++]; }
}

/* ********************************************************************************************* */
/// Computes the imu value from the imu readings
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
		filter_kalman_t* kf) {

	// ======================================================================
	// Get the readings

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	_imu = atan2(newX, imu_msg->data[2]); 
	_imuSpeed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// ======================================================================
	// Filter the readings

	// Setup the data
	kf->z[0] = _imu, kf->z[1] = _imuSpeed;

	// Setup the time-dependent process matrix
	kf->A[0] = kf->A[3] = 1.0;
	kf->A[2] = dt;

	// Setup the process noise matrix
	static const double k1 = 2.0;
	static const double k1b = 5.0;
	kf->R[0] = (dt*dt*dt*dt) * k1 * (1.0 / 4.0);
	kf->R[1] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[2] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[3] = (dt*dt) * k1b;
	
	// First make a prediction of what the reading should have been, then correct it
	filter_kalman_predict(kf);
	filter_kalman_correct(kf);

	// Set the values
	_imu = kf->x[0], _imuSpeed = kf->x[1];
}
