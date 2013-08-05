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
 * @file 03-wheels.cpp
 * @author Can Erdogan
 * @date July 11, 2013
 * @brief This demonstration simple moves the wheels back and forth, and spin to show how to use 
 * of the wheels. To do so, we will the joystick for input. 
 */

#include <unistd.h>
#include <iostream>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

somatic_d_t daemon_cx;								///< The properties of this "daemon"
somatic_motor_t amc; 									///< The interface to the wheel motor group
ach_channel_t js_chan;								///< The ach channel to the joystick daemon

typedef Vector4d State;								///< The pos/vel of the left/right wheels: qL, qR, qL., qR.

/* ******************************************************************************************** */
/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector4d& refState) {

	// Set the velocities taking into account the forward and spin axis values
	static const double kMaxForwVel = 2.0, kMaxSpinVel = 3.0;
	refState(2) = kMaxForwVel * js_forw + kMaxSpinVel * js_spin;
	refState(3) = kMaxForwVel * js_forw - kMaxSpinVel * js_spin;

	// Integrate the reference positions with the current reference velocities
	refState(0) += dt * refState(2);
	refState(1) += dt * refState(3);
}

/* ******************************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick 
bool getJoystickInput(double& js_forw, double& js_spin) {

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return false;

	// Set the values for the axis
	js_forw = -js_msg->axes->data[1];
	js_spin = js_msg->axes->data[2];
	return true;
}

/* ******************************************************************************************** */
/// The continuous control loop which has 4 state variables, {x, x., psi, psi.}, where
/// x is for the position along the heading direction and psi is the heading angle. We create
/// reference x and psi values from the joystick and follow them with pd control.
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Initially the reference position and velocities are zero (don't move!)
	Vector4d refState, state;
	refState << 0.0, 0.0, 0.0, 0.0;

	// Unless an interrupt or terminate message is received, process the new message
	cout << "start..." << endl;
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	while(!somatic_sig_received) {

		bool debug = (c_++ % 1 == 0);

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Get the current state from the motor readings
		somatic_motor_update(&daemon_cx, &amc);
		state = Vector4d(amc.pos[0], amc.pos[1], amc.vel[0], amc.vel[1]);
		if(debug) cout << "\nstate:" << state.transpose() << endl;

		// Get the joystick input for the js_forw and js_spin axes
		double js_forw = 0.0, js_spin = 0.0;
		bool gotInput = false;
		while(!gotInput) gotInput = getJoystickInput(js_forw, js_spin);
		if(debug) printf("forw: %lf, spin: %lf\n", js_forw, js_spin);

		// Determine the reference values for x and psi
		updateReference(js_forw, js_spin, dt, refState);
		if(debug) cout << "refState:" << refState.transpose() << endl;
		
		// Compute the necessary current inputs to the wheels with the current and reference states
		static const double Kp = 2.0, Kd = 12.0; 
		double u [2];
		u[0] = -Kp * (state(0) - refState(0)) -Kd * (state(2) - refState(2));
		u[1] = -Kp * (state(1) - refState(1)) -Kd * (state(3) - refState(3));
		if(debug) printf("u: (%lf, %lf)\n", u[0], u[1]);

		// Set the motor velocities
		somatic_motor_cmd(&daemon_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2, NULL);
		usleep(1e3);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Initialize the motor and daemons
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "03-wheels";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motor group
	somatic_motor_init(&daemon_cx, &amc, 2, "amc-cmd", "amc-state");

	// Set the min and maximum position and velocity valid/limit values for motors
	double ** limits[] = { 
		&amc.pos_valid_min, &amc.vel_valid_min, &amc.pos_limit_min, &amc.vel_limit_min, 
		&amc.pos_valid_max,	&amc.vel_valid_max, &amc.pos_limit_max, &amc.vel_limit_max};
	for(size_t i=0; i<4; i++)  { aa_fset(*limits[i],-1024.1, 2); }
	for(size_t i=4; i<8; i++) { aa_fset(*limits[i],1024.1, 2); }

	// Update the motors to get the current values
	somatic_motor_update(&daemon_cx, &amc);
	usleep(1e5);

	// Set the offset values to the motor group so initial wheel pos readings are zero
	somatic_motor_update(&daemon_cx, &amc);
	double pos_offset[2] = {-amc.pos[0], -amc.pos[1]};
	aa_fcpy(amc.pos_offset, pos_offset, 2);
	usleep(1e5);

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
}

/* ******************************************************************************************** */
/// Send zero velocity to the motors and kill daemon. Also clean up daemon structs.
void destroy() {

	// Stop and kill the motors
	double vals[] = {0.0, 0.0};
	somatic_motor_cmd(&daemon_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, vals, 2, NULL);
	somatic_motor_destroy(&daemon_cx, &amc);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}
