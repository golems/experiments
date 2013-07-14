/**
 * @file 04-balancing.cpp
 * @author Munzir Zafar, Can Erdogan
 * @date July 11, 2013
 * @brief This demonstration shows the balancing of the robot while following the position
 * and velocity commands set by the joystick.
 * NOTE We are using a Vector6d to represent the state which has: th, th. x, x., psi, psi.
 * where . is the derivative, th is the imu angle, x is the distance along the heading and
 * psi is the heading angle.
 */

#include "helpers.h"

using namespace std;
using namespace dynamics;


/* ******************************************************************************************** */
// Offset values for FT sensing

Vector6d leftOffset; 
Vector6d leftWheelWrench;
Vector6d rightOffset;
Vector6d rightWheelWrench;

/* ******************************************************************************************** */
/// The continuous control loop which has 4 state variables, {x, x., psi, psi.}, where
/// x is for the position along the heading direction and psi is the heading angle. We create
/// reference x and psi values from the joystick and follow them with pd control.
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Read the FT sensor wrenches, shift them on the wheel axis and display
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	while(!somatic_sig_received) {

		myDebug = false & (c_++ % 30 == 0);

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Update imu
		double imu, imuSpeed;
		getImu(&imuChan, imu, imuSpeed, dt, kf);
	
		// Update joint values
		somatic_motor_update(&daemon_cx, &llwa);
		somatic_motor_update(&daemon_cx, &rlwa);
		somatic_motor_update(&daemon_cx, &waist);

		// update the robot
		updateDart(imu);

		if(myDebug) cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
	
		// Get wrenches on the two arms in world frame and shift them to find the wrench on the wheel
		Vector6d raw, leftFTWrench, rightFTWrench;
		if(getFT(daemon_cx, left_ft_chan, raw)) {
			computeExternal(raw + leftOffset, *robot, leftFTWrench, true);
			computeWheelWrench(leftFTWrench, *robot, leftWheelWrench, true);
		} 
		if(getFT(daemon_cx, right_ft_chan, raw)) {
			computeExternal(raw + rightOffset, *robot, rightFTWrench, false);
			computeWheelWrench(rightFTWrench, *robot, rightWheelWrench, false);
		} 
		cout << leftWheelWrench (4) << endl;
		//if(myDebug) cout << "right Wh: " << rightWheelWrench.transpose() << endl;
		
		// Get resulting wrench on the wheel
		Vector6d wheelWrench;
		wheelWrench = leftWheelWrench + rightWheelWrench;
		if(myDebug) cout << "total   : " << wheelWrench.transpose() << endl << endl;
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Initialize the motor and daemons
void init() {

	// =======================================================================
	// Initialize the daemon, imu/joystick channels and kalman filter

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "06-modelExternal";
	somatic_d_init(&daemon_cx, &dopt);

	// Open the channel for imu
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	
	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// =======================================================================
	// Initialize the motors

	// Initialize the motor groupis and imu
	somatic_motor_init(&daemon_cx, &amc, 2, "amc-cmd", "amc-state");
	somatic_motor_init(&daemon_cx, &waist, 2, "waist-cmd", "waist-state");
	somatic_motor_init(&daemon_cx, &llwa, 7, "llwa-cmd", "llwa-state");
	somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
	
	// Set the min and maximum position and velocity valid/limit values for motors
	double ** limits[] = { 
		&amc.pos_valid_min, &amc.vel_valid_min, &amc.pos_limit_min, &amc.vel_limit_min, 
		&waist.pos_valid_min, &waist.vel_valid_min, &waist.pos_limit_min, &waist.vel_limit_min, 
		&llwa.pos_valid_min, &llwa.vel_valid_min, &llwa.pos_limit_min, &llwa.vel_limit_min, 
		&rlwa.pos_valid_min, &rlwa.vel_valid_min, &rlwa.pos_limit_min, &rlwa.vel_limit_min, 
		&amc.pos_valid_max,	&amc.vel_valid_max, &amc.pos_limit_max, &amc.vel_limit_max,
		&waist.pos_valid_max,	&waist.vel_valid_max, &waist.pos_limit_max, &waist.vel_limit_max,
		&rlwa.pos_valid_max,	&rlwa.vel_valid_max, &rlwa.pos_limit_max, &rlwa.vel_limit_max,
		&llwa.pos_valid_max,	&llwa.vel_valid_max, &llwa.pos_limit_max, &llwa.vel_limit_max};
	for(size_t i=0; i<16; i++)  { aa_fset(*limits[i],-1024.1, (i < 8) ? 2 : 7); }
	for(size_t i=16; i<32; i++) { aa_fset(*limits[i],1024.1, (i < 24) ? 2 : 7); }

	// Reset motors
	double zeros2[2] = {0.0, 0.0}, zeros7[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
	somatic_motor_cmd(&daemon_cx, &waist, SOMATIC__MOTOR_PARAM__MOTOR_RESET, zeros2, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, zeros7, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, zeros7, 7, NULL);
	usleep(1e5);
	
	// Update the motors to get the current values
	somatic_motor_update(&daemon_cx, &amc);
	somatic_motor_update(&daemon_cx, &waist);
	somatic_motor_update(&daemon_cx, &rlwa);
	somatic_motor_update(&daemon_cx, &llwa);
	usleep(1e5);

	// Get imu data
	double imu = 0.0, imuSpeed;
	for(int i = 0; i < 500; i++) {
		double temp;
		getImu(&imuChan, temp, imuSpeed, 0.0, NULL); 
		imu += temp;
	}
	imu /= 500;
	cout << "imu : " << imu*180.0/M_PI << endl;

	// Set the offset values to amc motor group so initial wheel pos readings are zero
	somatic_motor_update(&daemon_cx, &amc);
	double pos_offset[2] = {-amc.pos[0]-imu, -amc.pos[1]-imu};
	aa_fcpy(amc.pos_offset, pos_offset, 2);
	usleep(1e5);

	// Initialize kalman filter for the imu and set the measurement and meas. noise matrices
	// Also, set the initial reading to the current imu reading to stop moving from 0 to current
	kf = new filter_kalman_t;
	filter_kalman_init(kf, 2, 0, 2);
	kf->C[0] = kf->C[3] = 1.0;
	kf->Q[0] = kf->Q[3] = 1e-3;
	kf->x[0] = imu, kf->x[1] = imuSpeed;

	// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
	system("sns -k lft");
	usleep(20000);
	system("netcanftd -v -d -I lft -b 1 -B 1000 -c llwa_ft -k -r");

	// Open the state and ft channels
	somatic_d_channel_open(&daemon_cx, &left_ft_chan, "llwa_ft", NULL);
	somatic_d_channel_open(&daemon_cx, &right_ft_chan, "rlwa_ft", NULL);

	// Get the first force-torque reading and compute the offset with it
	cout << "reading FT now" << endl;
	Vector6d left_ft_data, right_ft_data, temp;
	left_ft_data << 0,0,0,0,0,0;
	right_ft_data << 0,0,0,0,0,0;
	for(size_t i = 0; i < 1e3; i++) {
		// Left Arm
		bool gotReading = false;
		while(!gotReading) gotReading = getFT(daemon_cx, left_ft_chan, temp);
		left_ft_data += temp;
		// Right Arm
		gotReading = false;
	//	while(!gotReading) gotReading = getFT(daemon_cx, right_ft_chan, temp);
		right_ft_data += temp;
	}
	left_ft_data /= 1e3;
	right_ft_data /= 1e3;
	cout << "waist <" << waist.pos[0] << "," << waist.pos[1] << ">: " << (waist.pos[0]-waist.pos[1])/2.0 << endl;
	computeOffset(imu, (waist.pos[0]-waist.pos[1])/2.0, llwa, left_ft_data, *robot, leftOffset, true);
	computeOffset(imu, (waist.pos[0]-waist.pos[1])/2.0, rlwa, right_ft_data, *robot, rightOffset, false);
}

/* ******************************************************************************************** */
/// Send zero velocity to the motors and kill daemon. Also clean up daemon structs.
void destroy() {

	// Close imu channel
	somatic_d_channel_close(&daemon_cx, &imuChan);

	// Open the state and ft channels
	somatic_d_channel_close(&daemon_cx, &left_ft_chan);
	somatic_d_channel_close(&daemon_cx, &right_ft_chan);
	
	// Stop the motors
	double zeros2[2] = {0.0, 0.0}, zeros7[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
	somatic_motor_cmd(&daemon_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, zeros2, 2, NULL);
	somatic_motor_cmd(&daemon_cx, &waist, SOMATIC__MOTOR_PARAM__MOTOR_HALT, zeros2, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, zeros7, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, zeros7, 7, NULL);

	// Stop and kill the schunk motors
	somatic_motor_destroy(&daemon_cx, &amc);
	somatic_motor_destroy(&daemon_cx, &waist);
	somatic_motor_destroy(&daemon_cx, &llwa);
	somatic_motor_destroy(&daemon_cx, &rlwa);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// Load the world and the robot
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton(0);

	// Initialize, run, destroy
	init();
	run();
	destroy();
	return 0;
}
