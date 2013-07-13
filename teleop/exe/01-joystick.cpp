/**
 * @file 01-joystick.cpp
 * @author Can Erdogan
 * @date July 13, 2013
 * @brief This file demonstrates the use of the Logitech joystick controller to control the Schunk
 * modules and Robotiq grippers.
 */

#include "initModules.h"
#include "motion.h"

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY

/* ********************************************************************************************** */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t state_chan;
somatic_motor_t llwa, rlwa, torso;

char* b;
double* x;

/* ********************************************************************************************* */
/// Reads the joystick data into global variables 'b' and 'x'
void readJoystick() {

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

	// Get the values
	for(size_t i = 0; i < 10; i++) 
		b[i] = js_msg->buttons->data[i] ? 1 : 0;
	memcpy(x, js_msg->axes->data, sizeof(x));
	
	// Free the joystick message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
}

/* ********************************************************************************************* */
/// Controls the torso module with the 8-pad left/right b
void controlTorso() {
	
}

/* ********************************************************************************************* */
/// Controls the arms
void controlArms () {

	// Check the b for each arm and apply velocities accordingly
	// For left: 4 or 6, for right: 5 or 7, lower arm button is smaller (4 or 5)
	somatic_motor_t* arm [] = {&llwa, &rlwa};
	for(size_t arm_idx = 0; arm_idx < 2; arm_idx++) {

		// Initialize the input
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Change the input based on the lower or higher button input
		bool inputSet = true;
		size_t lowerButton = 4 + arm_idx, higherButton = 6 + arm_idx;
		if(b[lowerButton] && !b[higherButton]) memcpy(&dq[4], x, 3*sizeof(double));
		else if(!b[lowerButton] && b[higherButton]) memcpy(dq, x, 4*sizeof(double));
		else inputSet = false;
		
		// Set the input for this arm
		if(inputSet) somatic_motor_cmd(&daemon_cx, arm[arm_idx], VELOCITY, dq, 7, NULL);
	}
}

/* ********************************************************************************************* */
/// Continuously process the joystick data and sets commands to the modules
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Read the joystick data 
		readJoystick();

		// Control the arms if necessary
		controlArms();

		// Control the torso
		double dq [] = {1.0};
		somatic_motor_cmd(&daemon_cx, &torso, VELOCITY, dq, 1, NULL);

		if(fabs(x[4]) > 0.1) somatic_motor_cmd(&daemon_cx, &torso, VELOCITY, &(x[4]), 1, NULL);
	
		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void init () {
	
	// Initialize space for the button and axes
	b = new char [10];
	x = new double [6];

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-joystick";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the arms and the torso module
	initArm(daemon_cx, llwa, "llwa");
	initArm(daemon_cx, rlwa, "rlwa");
	initTorso(daemon_cx, torso);

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
}

/* ********************************************************************************************* */
void destroy() {

	// Halt the Schunk modules
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
int main() {

	init();
	run();
	destroy();

	return 0;
}
