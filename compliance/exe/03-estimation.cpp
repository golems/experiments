/**
 * @file 03-estimation.cpp
 * @author Can Erdogan
 * @date June 15, 2013
 * @brief This file demonstrates how to estimate the external forces from the ft values by
 * removing the effect of the end-effector in the readings. We remove the drift/bias in
 * the values as shown in 02-correction.cpp.
 * See the FTEstimation report at @thebrain:/home/git/krang/Reports.
 * Note that the F/T readings are in Nm's (Newton-meters).
 */

#include "helpers.h"

using namespace std;
using namespace dynamics;
using namespace simulation;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;
ach_channel_t imuChan;
ach_channel_t waistChan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;
Vector6d offset;							///< the offset we are going to decrease from raw readings

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	Vector6d raw, external;
	Matrix3d Rsb;	//< The sensor frame in bracket frame (only rotation)
	double waist=0.0, imu=0.0;	
	while(!somatic_sig_received) {
		
		c++;

		// Move the arm to any position with the joystick
		setJoystickInput(daemon_cx, js_chan, llwa, llwa);
		somatic_motor_update(&daemon_cx, &llwa);

		// Get imu/waist data
		getImu(&imu, imuChan);
		getWaist(&waist, waistChan);
		
		// Get the f/t sensor data and compute the ideal value
		size_t k = 50;
		bool result = (c % k == 0) && getFT(daemon_cx, ft_chan, raw);
		if(!result) continue;

		// Compute the ideal value
		Vector6d ideal = raw + offset;

		// Compute the external forces from ideal readings
		computeExternal(imu, waist, llwa, ideal, *(mWorld->getSkeleton(0)), external);
		pv(external);

		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_d_channel_close(&daemon_cx, &waistChan);
	somatic_d_channel_close(&daemon_cx, &imuChan);
	
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init(daemon_cx, js_chan, imuChan, waistChan, ft_chan, llwa, offset);
	cout << "Initialization done!" << endl;
	run();
	destroy();
	return 0;
}
