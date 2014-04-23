/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file 01-joystick.cpp
 * @author Can Erdogan
 * @date July 13, 2013
 * @brief This file demonstrates the use of the Logitech joystick controller to control the Schunk
 * modules and Robotiq grippers.
 */

#include "initModules.h"
#include "motion.h"
#include <iostream>

using namespace std;

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION

/* ********************************************************************************************** */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t state_chan;
ach_channel_t waistCmdChan;
somatic_motor_t llwa, rlwa, torso;

Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		///< The waist command

char b [10];
double x [6];

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
// The preset arm configurations: forward, thriller, more forward for balancing, zero
double presetArmConfs [][7] = {
  {  1.102, -0.589,  0.000, -1.339,  0.000, -1.400,  0.000},
  { -1.102,  0.589,  0.000,  1.339,  0.000,  1.400,  0.000},
  {  1.008,  -1.113,  -0.000,  -1.594,   0.037,   1.022,  -0.021}, // sitting grasp ready (left)
  {  -0.855,   1.113,   0.000,   1.561,  -0.028,  -1.052,   0.021}, // sitting grasp ready (right)
  {  1.400, -1.000,  0.000, -0.800,  0.000, -0.800,  0.000}, 
  { -1.400,  1.000,  0.000,  0.800,  0.000,  0.800,  0.000},
  {  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
  {  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
};

/* ********************************************************************************************* */
/// Controls the arms
void controlArms () {

	// Return if the x[3] is being used for robotiq hands
	if(fabs(x[3]) > 0.2) return;

	// Check if one of the preset configurations are requested by pressing 9 and
	// any of the buttons from 1 to 4 at the same time
	if(((b[4] == 1) && (b[6] == 1)) || ((b[5] == 1) && (b[7] == 1))) {

		// Check if the button is pressed for the arm configuration is pressed, if so send pos commands
		bool noConfs = true;
		for(size_t i = 0; i < 4; i++) {
			if(b[i] == 1) {
				if((b[4] == 1) && (b[6] == 1)) 
					somatic_motor_cmd(&daemon_cx, &llwa, POSITION, presetArmConfs[2*i], 7, NULL);
				if((b[5] == 1) && (b[7] == 1)) 
					somatic_motor_cmd(&daemon_cx, &rlwa, POSITION, presetArmConfs[2*i+1], 7, NULL);
				noConfs = false; 
				return;
			}
		}
		
		// If nothing is pressed, stop the arms
		if(noConfs) {
			double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			somatic_motor_cmd(&daemon_cx, &llwa, VELOCITY, dq, 7, NULL);
			somatic_motor_cmd(&daemon_cx, &rlwa, VELOCITY, dq, 7, NULL);
			return;
		}
	}
	
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
/// Handles the joystick commands for the waist module
void controlWaist() {

	// Set the mode we want to send to the waist daemon
	Somatic__WaistMode waistMode;
	if(x[5] < -0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
	else if(x[5] > 0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
	else waistMode = SOMATIC__WAIST_MODE__STOP;

	// Send message to the krang-waist daemon
	somatic_waist_cmd_set(waistDaemonCmd, waistMode);
	int r = SOMATIC_PACK_SEND(&waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)));
}

/* ********************************************************************************************* */
/// Control the Robotiq hands - button 5 or 6 sets left/right, axis 3 sets the direction
/// TODO: Find a better joystick to command mapping. The results are not very intuitive.
void controlRobotiq() {

	// Return if x[2] is being used for the L7/R7 motors
	if(fabs(x[2]) > 0.2) return;

	// Skip some loops
	static int counter = 0;
	if(counter++ % 20 != 0) return;

	// For each hand, check if the button is pressed and if so give the command
	for(size_t i = 0; i < 2; i++) {

		// Turn the axis value between [-1, 1], to [0,255] - if 0, do nothing.
		int val = fmax(0, fmin(255, ((x[3] + 1.0) / 2.0) * 256));

		// Write the command line command and execute it
		char buf [256];
		if((b[4 + i] == 1) && (fabs(x[3]) > 0.05))  {
			sprintf(buf, "robotiq_cmd b gp%x -c %cgripper-cmd", val, i ? 'r' : 'l');
			printf("'%s'\n", buf);
			system(buf);
		}
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
		double dq [] = {x[4] / 7.0};
		somatic_motor_cmd(&daemon_cx, &torso, VELOCITY, dq, 1, NULL);

		// Control the waist
		controlWaist();

		// Control the robotiq hands
		controlRobotiq();

		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);	
		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void init () {
	
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

	// Initialize the waist channel
	somatic_d_channel_open(&daemon_cx, &waistCmdChan, "waistd-cmd", NULL);
}

/* ********************************************************************************************* */
void destroy() {

	// Halt the waist modules
	static Somatic__WaistCmd *waistDaemonCmd=somatic_waist_cmd_alloc();
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
	somatic_metadata_set_time_now(waistDaemonCmd->meta);
	somatic_metadata_set_until_duration(waistDaemonCmd->meta, .1);
	ach_status_t r = SOMATIC_PACK_SEND( &waistCmdChan, somatic__waist_cmd, waistDaemonCmd );
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", ach_result_to_string(r));
	somatic_d_channel_close(&daemon_cx, &waistCmdChan);

	// Halt the Schunk modules
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	//
	cout << "Destroyed and halted" << endl;
}

/* ********************************************************************************************* */
int main() {

	init();
	run();
	destroy();

	return 0;
}
