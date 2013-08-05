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
 * @file 01-gripperWeight.cpp
 * @author Can Erdogan
 * @date June 12, 2013
 * @brief This file demonstrates the effect of the weight on the gripper readings. The problem is
 * that the instruction to use the f/t sensor readings tell us to subtract the initial reading
 * from the following. However, the initial reading incorporates the mass after the f/t sensor
 * which induces a specific force and torque for that initial kinematic setup. As we move the
 * arm, that effect is still being subtracted although it is not there anymore. 
 * To show this phenomenon, we are going to rotate the last joint of the arm when it is straight
 * out in front of it. In this state, the f/t sensor should measure the same total force in 
 * whichever configuration it is because the end-effector is symmetric around the joint (the tipped
 * one). However, we will observe it doubles as we reach a 180 degree rotation.
 * A side note: We need to restart the netcanft, the force/torque sensor driver which performs
 * the initial subtraction, every time we run this code. We will do this within the code with a
 * 'system' command.
 * NOTE 2: We assume the experiment is done on the left arm and the initial service call (krang/
 * arms-pedestal) is already done to create the channels.
 * NOTE 3: You may need to change the constant for the canbus number for the f/t sensor.
 */

#include "helpers.h"
#include "initModules.h"
#include "motion.h"

using namespace std;

#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
	llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);

#define move(); {double q [] = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0, 2.0*M_PI}; \
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_POSITION, q, 7, NULL); continue;}

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;

/* ********************************************************************************************** */
void init (){

	/// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
	system("killall -s 9 netcanftd");
	usleep(20000);
	system("/home/cerdogan/Documents/Software/drivers/netcanft/build/netcanftd -v -v -d -I lft -b 2 -B 1000 -c llwa_ft");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-gripperWeight";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the left arm
	initArm(daemon_cx, llwa, "llwa");

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open the state and ft channels 
	somatic_d_channel_open(&daemon_cx, &ft_chan, "llwa_ft", NULL);
}

/* ********************************************************************************************* */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	Vector6d ft_data;
	double dq7 = -0.7;
	bool changed = false;
	double lastCheckPoint = 9.14;
	double dqStop [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	while(!somatic_sig_received) {
		
		if((c++ % 100000) != 0) continue;

		// Check if the arm reached -2pi, if so, turn it around.
		somatic_motor_update(&daemon_cx, &llwa);
		continue;
		if(!changed && (llwa.pos[6] < -2*M_PI)) {
			dq7 = -dq7;
			changed = true;
		}
	
		// Check if close to a check point
/*
		double mod = fmod(fabs(llwa.pos[6]), M_PI_2/2.0);
		if((mod < 1e-2) && (fabs(lastCheckPoint - fabs(llwa.pos[6])) > 0.3)) {
			lastCheckPoint = fabs(llwa.pos[6]);
			cout << "Reached checkpoint: " << llwa.pos[6] << endl;
			somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dqStop, 7, NULL);
			somatic_motor_update(&daemon_cx, &llwa);
			usleep(1e7);
			cout << "\tstarting again..." << endl;
		}
*/
		
		// Move the arm at the given velocity
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, dq7};
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);

		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
/// Kills the motor and daemon structs 
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
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
