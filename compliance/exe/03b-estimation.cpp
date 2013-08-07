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
 * @file 03-estimation.cpp
 * @author Can Erdogan
 * @date June 15, 2013
 * @brief This file demonstrates how to estimate the external forces from the ft values by
 * removing the effect of the end-effector in the readings. We remove the drift/bias in
 * the values as shown in 02-correction.cpp.
 * See the FTEstimation report at @thebrain:/home/git/krang/Reports.
 * Note that the F/T readings are in Nm's (Newton-meters).
 */

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kore.hpp>

using namespace std;
using namespace Krang;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
simulation::World* world = NULL;
Krang::Hardware* hw;

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	while(!somatic_sig_received) {
		
		// Get the time that has passed
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Update the sensor readings and the kinematics
		c++;
		hw->updateSensors(dt);

		// Print the external f/t readings for the left arm
		if(c % 10 == 1) {
			hw->printState();
			std::cout << "lft ext: " << hw->fts[Krang::LEFT]->lastExternal.transpose() << "\n" << std::endl;
		}
		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}


/* ******************************************************************************************** */
void init () {

	// Get the world
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "03b-estimation";
	somatic_d_init( &daemon_cx, &dopt );

	// Get the robot
	hw = new Krang::Hardware((Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL & ~Krang::Hardware::MODE_GRIPPERS),
	                          &daemon_cx, world->getSkeleton("Krang"));
	std::cout << "Initialization done!" << std::endl;
}

/* ******************************************************************************************** */
/// The main thread
int main(const int argc, char** argv) {

	init();
	run();
	somatic_d_destroy(&daemon_cx);
	return 0;
}
