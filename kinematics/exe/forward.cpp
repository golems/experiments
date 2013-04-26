/**
 * @file forwardKinematics.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date April 24, 2012
 * @briefs This file receives the position of the red dot detected from the
 * vision PC and predicts the loaction of the red dot by forward kinematics
 * and then compares the two to determine the accuracy of the forward 
 * kinematics.
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

#include <schkin.h>
#include <amino.h>
#include <iostream>


/* ********************************************************************************************** */

somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t state_chan;

// The ach channel and its name to receive information from vision PC
ach_channel_t chan_transform;

somatic_motor_t llwa, rlwa;

/* ********************************************************************************************* */
void init() {
	
	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "bal-hack";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the motors with the daemon context, channel names and # of motors
	somatic_motor_init(&daemon_cx, &llwa, 7, "llwa-cmd", "llwa-state");
	somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&llwa.pos_valid_min, &rlwa.pos_valid_min, &llwa.vel_valid_min, &rlwa.vel_valid_min, 
		&llwa.pos_limit_min, &rlwa.pos_limit_min, &llwa.pos_limit_min, &rlwa.pos_limit_min, 
		&llwa.pos_valid_max, &rlwa.pos_valid_max, &llwa.vel_valid_max, &rlwa.vel_valid_max, 
		&llwa.pos_limit_max, &rlwa.pos_limit_max, &llwa.pos_limit_max, &rlwa.pos_limit_max};
	for(size_t i = 0; i < 8; i++) aa_fset(*limits[i], -1024.1, 7);
	for(size_t i = 8; i < 16; i++) aa_fset(*limits[i], 1024.1, 7);
	
	// Update and reset them
	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
	somatic_motor_update(&daemon_cx, &rlwa);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);

	// Open joystick channel
	int r  = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK,
				   "Ach failure %s on opening Joystick channel (%s, line %d)\n",
				   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	usleep(10000);

	// --------- open channels ----------
	somatic_d_channel_open(&daemon_cx, &state_chan, "krang-state", NULL);
	somatic_d_channel_open(&daemon_cx, &chan_transform, "chan_transform", NULL);	

}

/* ********************************************************************************************* */
void updateRedDot() {

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(1));
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &chan_transform, &numBytes, &abstimeout, 
		ACH_O_LAST, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return;

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return;
	if(msg->meta->type != SOMATIC__MSG_TYPE__TRANSFORM) return; 

	// Read the force-torque message
	Somatic__Transform* message = somatic__transform__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	printf("[server] transform:\t");
	for(size_t i = 0; i < 3; i++)
		printf("%6.2f  ", message->translation->data[i]); 
	for(size_t i = 0; i < 4; i++)
		printf("%6.2f  ", message->rotation->data[i]); 
	printf("\n"); fflush(stdout);
}

/* ********************************************************************************************* */
/// Set motor values with joystick input
void setJoystickInput () {

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

	// Get the values
	char b [10];
	float x [6];
	for(size_t i = 0; i < 10; i++) 
		b[i] = js_msg->buttons->data[i] ? 1 : 0;
	memcpy(x, js_msg->axes->data, sizeof(x));
	
	// Check the buttons for each arm and apply velocities accordingly
	// For left: 4 or 6, for right: 5 or 7, lower arm button is smaller (4 or 5)
	somatic_motor_t* arm [] = {&llwa, &rlwa};
	for(size_t arm_idx = 0; arm_idx < 2; arm_idx++) {

		// Initialize the input
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Change the input based on the lower or higher button input
		size_t lowerButton = 4 + arm_idx, higherButton = 6 + arm_idx;
		if(b[lowerButton] && !b[higherButton]) memcpy(&dq[4], x, 3*sizeof(double));
		else if(!b[lowerButton] && b[higherButton]) memcpy(dq, x, 4*sizeof(double));

		// Set the input for this arm
		somatic_motor_cmd(&daemon_cx, arm[arm_idx], SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
	}
	
	// Free the joystick message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
}

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Receive the 3-D coordinate of the red dot from vision PC
		updateRedDot();
		
		// Get the motor positions
		somatic_motor_update(&daemon_cx, &llwa);
		somatic_motor_update(&daemon_cx, &rlwa);

		// Read the joystick data and send the input velocities to the arms
		setJoystickInput();

		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);

	somatic_d_channel_close(&daemon_cx, &chan_transform);
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
int main() {


	init();
	run();
	destroy();

	return 0;
}
