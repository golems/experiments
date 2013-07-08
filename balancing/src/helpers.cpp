/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and filtering...
 */

/* ********************************************************************************************* */
/// Filters the imu (q2), wheel (q1_0 & q1_1) and waist (q3) position and velocities.
void filterState (filter_kalman_t* kf, double& q1_0, double& dq1_0, double& q1_1, double& dq1_1, 
		double& q2, double& dq2, double& q3, double& dq3) {

	// Set the data of the filter (note the wheel and imu positions are added).
	kf->z[0] = q2; kf->z[1] = dq2;
	kf->z[2] = q1_0 + q2; kf->z[3] = dq1_0 + dq2;
	kf->z[4] = q1_1 + q2; kf->z[5] = dq1_1 + dq2;
	kf->z[6] = q3; kf->z[7] = dq3;

}

/* ********************************************************************************************* */
/// Computes the imu value from the imu readings
double getImu () {

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Prepare the ssdmu structure 
	ssdmu_sample_t imu_sample;
	imu_sample.x  = imu_msg->data[0];
	imu_sample.y  = imu_msg->data[1];
	imu_sample.z  = imu_msg->data[2];
	imu_sample.dP = imu_msg->data[3];
	imu_sample.dQ = imu_msg->data[4];
	imu_sample.dR = imu_msg->data[5];

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// Make the calls to extract the pitch and rate of extraction
	return -ssdmu_pitch(&imu_sample) + M_PI_2;				 
}



