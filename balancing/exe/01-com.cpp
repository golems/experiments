/**
 * @file 01-com.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This executable shows how to estimate the center of mass of the Krang.
 */

/* ******************************************************************************************** */
// Communication and filtering

filter_kalman_t *kf;						///< The kalman filter to smooth the imu readings
somatic_d_t daemon_cx;
ach_channel_t amcChan;
ach_channel_t imuChan;
ach_channel_t waistChan;			///< the state channel for the waist module
ach_channel_t leftArmChan;		///< the state channel for the left arm modules
ach_channel_t rightArmChan;		///< the state channel for the right arm modules

/* ********************************************************************************************* */
/// Gets the data from the channels
void getData (Somatic__MotorState** waist, Somatic__MotorState** leftArm, Somatic__MotorState** 
		rightArm, Somatic__MotorState** amc, Somatic__ForceMoment** leftFt, 
		Somatic__ForceMoment** rightFt, double* imu) {

	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);

	// Get the data from the motors
	int r;
	assert(((*waist = getMotorMessage(waistChan)) != NULL) && "Waist call failed");
	assert(((*amc = getMotorMessage(amcChan)) != NULL) && "Wheels call failed");
	assert(((*leftArm = getMotorMessage(leftArmChan)) != NULL) && "leftArm call failed");
	assert(((*rightArm = getMotorMessage(rightArmChan)) != NULL) && "rightArm call failed");

	// Get the data from imu
	*imu = getImu ();

	// Get the data from ft sensors
	*leftFt = SOMATIC_WAIT_LAST_UNPACK( r, somatic__force_moment, 
		&protobuf_c_system_allocator, 1024, &ftLeftChan, &abstime);
	*rightFt = SOMATIC_WAIT_LAST_UNPACK( r, somatic__force_moment, 
		&protobuf_c_system_allocator, 1024, &ftRightChan, &abstime);
}

/* ******************************************************************************************** */
void run() {

	// Send the event massage
	somatic_d_event(&krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing the joint values and computing the com until sigterm received
	while (!somatic_sig_received ) {

	}

	// Once the main loop returns, the program is done. Send the stopping event message.
	somatic_d_event(&krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
/// Initializes the ach channels needed to retrieve the state of the robot
void init () {

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-com";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the channels to the sensors
	somatic_d_channel_open(&daemon_cx, &amcChan, "amc-state", NULL);
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	somatic_d_channel_open(&daemon_cx, &leftArmChan, "llwa-state", NULL);
	somatic_d_channel_open(&daemon_cx, &rightArmChan, "rlwa-state", NULL);
}

/* ********************************************************************************************* */
/// Destroy the channel and daemon resources
void destroy () {
	somatic_d_channel_close(&daemon_cx, &imuChan);
	somatic_d_channel_close(&daemon_cx, &waistChan);	 
	somatic_d_channel_close(&daemon_cx, &leftArmChan); 
	somatic_d_channel_close(&daemon_cx, &rightArmChan);
	somatic_d_channel_close(&daemon_cx, &ftLeftChan);
	somatic_d_channel_close(&daemon_cx, &ftRightChan);
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}
