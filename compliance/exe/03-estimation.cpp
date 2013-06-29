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
somatic_motor_t lwa;
Vector6d offset;							///< the offset we are going to decrease from raw readings

/* ********************************************************************************************* */
// Argument processing

/// Options that will be presented to the user
static struct argp_option options[] = {
		{"arm",'a', "arm", 0, "arm to work on (left/right)", 0},
		{0, 0, 0, 0, 0, 0}
};

/// The one-line explanation of the executable
static char doc[]= "allows user to correct positions of the pcio modules";

/// The parser function
static int parse_opt( int key, char *arg, struct argp_state *state) {
	(void) state; 

	// Make sure the input flag for motor group is set
	if(key != 'a') return 0;

	// Determine which arm to work on
	if(strcmp(strdup(arg), "left") == 0) arm = LEFT;
	else if(strcmp(strdup(arg), "right") == 0) arm = RIGHT;
	else {
		printf("Unidentifiable motor group!\n");
		exit(0);
	}
	return 0;
}

/// The argp structure to parse stuff
static struct argp argp = {options, parse_opt, NULL, doc, NULL, NULL, NULL };

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
		setJoystickInput(daemon_cx, js_chan, lwa, lwa);
		somatic_motor_update(&daemon_cx, &lwa);

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
		computeExternal(imu, waist, lwa, ideal, *(mWorld->getSkeleton(0)), external);
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
	somatic_motor_cmd(&daemon_cx, &lwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_d_channel_close(&daemon_cx, &waistChan);
	somatic_d_channel_close(&daemon_cx, &imuChan);
	
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main(const int argc, char** argv) {
	argp_parse (&argp, argc, argv, 0, NULL, NULL);
	init(daemon_cx, js_chan, imuChan, waistChan, ft_chan, lwa, offset);
	cout << "Initialization done!" << endl;
	run();
	destroy();
	return 0;
}
