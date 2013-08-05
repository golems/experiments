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
 * @file simpleSineWheelcpp
 * @author Munzir Zafar
 * @date July 15, 2013
 * @brief Just a simple code to test the delays between the commanded current sent to
 * the wheel and the actual current in repsonse
 */

#include <dirent.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

somatic_d_t daemon_cx;				///< The properties of this "daemon"
somatic_motor_t amc; 					///< The interface to the wheel motor group

double Amplitude = 2.0;
double freq = 0.5;

int main (int argc, char * argv[]) {

	// Read amp and freq from cmd line	 
	assert(argc > 2 && "Usage: ./00-simpleSineCurrent <amplitude> <freq>");
	Amplitude = atof(argv[1]);
	freq = atof(argv[2]);
	
	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "wheelSine";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the amc	
	somatic_motor_init(&daemon_cx, &amc, 2, "amc-cmd", "amc-state");
	double ** limits[] = { 
		&amc.pos_valid_min, &amc.vel_valid_min, &amc.pos_limit_min, &amc.vel_limit_min,
		&amc.pos_valid_max,	&amc.vel_valid_max, &amc.pos_limit_max, &amc.vel_limit_max};
	for(size_t i=0; i<4; i++)  { aa_fset(*limits[i],-1024.1, 2); }
	for(size_t i=4; i<8; i++) { aa_fset(*limits[i],1024.1, 2); }

	// Send sine commands on the anc cmd channel and read the resulting current state
	struct timespec t_now, t_prev = aa_tm_now();
	double time = 0.0;
	static int c = 0;
	double u;
	while(!somatic_sig_received) {

		c++;
	
		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;
		time += dt;
		
		// Generate trhe current command for the wheels
		u = Amplitude * sin(2*M_PI*freq*time);
		
		// Sned the command
		double input[] = {u, u};
		somatic_motor_cmd(&daemon_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
		usleep(10e3);

		// Read the data from the amc
		somatic_motor_update(&daemon_cx, &amc);
		//if(c % 10 == 0) 
		printf("%lf\t%lf\t%lf\n", u, amc.cur[0], time);
	}

	// Stop the motor
	double input[] = {0, 0};
	somatic_motor_cmd(&daemon_cx, &amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
	
	// Destroy
	somatic_motor_destroy(&daemon_cx, &amc);
	somatic_d_destroy(&daemon_cx);
	return 0;	
}
