/**
 * @file 02-correction.cpp
 * @author Can Erdogan
 * @date June 14, 2013
 * @brief This file demonstrates how to compute the estimate what the raw force/torque reading 
 * should be when there are no external forces (only the gripper weight) and the offset that
 * needs to be decreased from future raw values.
 * The output should show that the ideal values reflect the mass of the ee and as you rotate
 * the wrist, the values should change accordingly.
 * See the FTEstimation report at @thebrain:/home/git/krang/Reports.
 * Note that the F/T readings are in Nm's (Newton-meters).
 */

#include "helpers.h"

using namespace std;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
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
	Vector6d raw, ideal;
	while(!somatic_sig_received) {
		
		c++;

		// Move the arm to any position with the joystick
		setJoystickInput(daemon_cx, js_chan, llwa, llwa);
		somatic_motor_update(&daemon_cx, &llwa);
	
		// Get the f/t sensor data and compute the ideal value
		bool result = (c % 1000000 == 0) && getFT(daemon_cx, ft_chan, raw);
		if(!result) continue;
		ideal = raw + offset;
		if(c % 1000000 == 0) {
			cout << ideal.transpose() << " " << llwa.pos[6] << " ";
			cout << sqrt(ideal(0)*ideal(0) + ideal(1)*ideal(1)) << endl;
		}
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init(daemon_cx, js_chan, ft_chan, llwa, offset);
	run();
	destroy();
	return 0;
}
