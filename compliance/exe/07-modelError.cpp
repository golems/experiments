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
 * @file 07-modelError.cpp
 * @author Can Erdogan
 * @date June 15, 2013
 * @brief This executable aims at analyzing the modeling error observed in experiments 02 and 03,
 * and documented in debugging-amplitude folder under data. The main observation is that the
 * f/t sensor seems to return weird signals when there is no mass on it. We want to create a 
 * table of what these signals so we can subtract them in runtime, assuming that they stay a 
 * constant, which we have seen in experiment-1 of 03-estimation.
 */

#include "helpers.h"
#include <Eigen/StdVector>

using namespace std;
using namespace dynamics;
using namespace simulation;

#define DEG2RAD(x) (((x) / 180.0) * M_PI)
#define M_2PI (2 * M_PI)
#define NUM_READINGS 1e3
#define fix(x) ((fabs(x) < 1e-5) ? 0 : x)


/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;
Vector6d offset;							

vector <Vector3d, aligned_allocator<Vector3d> > goals;		///< Last 3 dof that the arm will visit
vector <Vector6d, aligned_allocator<Vector6d> > readings;	///< Mean readings at the goal locations

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0, currGoal = 0;
	Vector6d raw, external;
	Matrix3d Rsb;	//< The sensor frame in bracket frame (only rotation)
	double q [] = {0.0, -M_PI_2, 0.0, 0.0, 0.0, M_PI_2, 0.0};	
	size_t numReadings = 0;
	vector <Vector6d, aligned_allocator<Vector6d> > localReadings;
	localReadings.resize(NUM_READINGS);
	while(!somatic_sig_received) {
		
		c++;

		// Check if reached all the goals
		if(currGoal == goals.size()) {
			cout << "done!" << endl;
			break;
		}

		// Check if the arm has reached the desired position
		somatic_motor_update(&daemon_cx, &llwa);
		Vector3d error = Vector3d(llwa.pos[4], llwa.pos[5], llwa.pos[6]) - goals[currGoal];
		double maxError = error.cwiseAbs().maxCoeff();
		bool reached = (fabs(maxError) < 1e-2);

		// If reached the position, get enough readings to average over
		if(reached) {

			// Get a new reading
			bool gotReading = false;
			while(!gotReading) gotReading = getFT(daemon_cx, ft_chan, raw);
			localReadings[numReadings] = raw;	
			numReadings++;

			// Increment the goal index if you got enough readings
			if(numReadings >= NUM_READINGS) {

				// Average the readings
				Vector6d mean = Vector6d::Zero();
				for(size_t i = 0; i < NUM_READINGS; i++) mean += localReadings[i];
				mean /= NUM_READINGS;

				// Write the reading and the joint values to a file
				for(size_t i = 0; i < 3; i++) printf("% 5.5lf\t", goals[currGoal](i));
				for(size_t i = 0; i < 6; i++) printf("% 5.5lf\t", mean(i));
				printf("\n"); fflush(stdout);

				// Get the next goal
				currGoal++;
				numReadings = 0;
			}
		}

		// If did not reach, move towards the goal
		for(size_t i = 0; i < 3; i++) q[i+4] = (goals[currGoal])(i);
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_POSITION, q, 7, NULL);
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
/// Generates the joint positions to reach discretized orientations of the end-effector
void generateJointValues () {

	goals.push_back(Vector3d(0.0, M_PI_2, 0.0));
	goals.push_back(Vector3d(1.04720, 1.57080, 6.28319));
	goals.push_back(Vector3d(1.04720, 1.57080, 5.75959));
	goals.push_back(Vector3d(1.04720, 1.57080, 5.23599));
	goals.push_back(Vector3d(1.04720, 1.57080, 4.71239));
	goals.push_back(Vector3d(1.04720, 1.57080, 4.18879));
	goals.push_back(Vector3d(1.04720, 1.57080, 3.66519));
	goals.push_back(Vector3d(1.04720, 1.57080, 3.14159));
	goals.push_back(Vector3d(1.04720, 1.57080, 2.61799));
	goals.push_back(Vector3d(1.04720, 1.57080, 2.09440));
	goals.push_back(Vector3d(1.04720, 1.57080, 1.57080));
	goals.push_back(Vector3d(1.04720, 1.57080, 1.04720));
	goals.push_back(Vector3d(1.04720, 1.57080, 0.52360));

	return;
	std::vector <int> arm_ids;		///< The index vector to set config of arms
	int arm_ids_a [] = {10, 12, 14, 16, 18, 20, 22};
	for(size_t i = 0; i < 7; i++) arm_ids.push_back(arm_ids_a[i]);

	// The separation between point samples where dth3 is the smallest angle
	double dth1 = -DEG2RAD(12.0), dth2 = DEG2RAD(10.0), dth3 = -DEG2RAD(30.0);
	int num_th1 = -M_PI / fabs(dth1), num_th2 = ((M_PI_2/9.0)*9.0) / fabs(dth2), num_th3 = M_2PI / fabs(dth3);
	printf("dth1: %lf, dth2: %lf, dth2: %lf\n", dth1, dth2, dth3);
	printf("num_th1: %lf, num_th2: %lf, num_th2: %lf\n", num_th1, num_th2, num_th3);

	goals.push_back(Vector3d(0.0, M_PI_2, 0.0));

	// We first fix the vertical joint - this basically determines the radius of the circle we
	// draw in the yz plane, starting from big to small
	bool turn1 = 1, turn3 = 0;
	for(int th2_idx = num_th2; th2_idx >= 0; th2_idx--) {

		// Compute th2 and decide on the direction th1 is going to rotate
		double th2 = th2_idx * dth2;
		dth1 *= -1;
		turn1 = !turn1;
		
		// Then, we move the biggest joint, th1, to create a circle for the ee position
		for(int th1_idx = 0; th1_idx < num_th1; th1_idx++) {

			// Compute th1 and decide on the direction th3 is going to rotate
			double th1 = (dth1 * th1_idx) + (turn1 ? 0.0 : M_2PI);
			dth3 *= -1;
			turn3 = !turn3;
	
			// If th1 and th3 are aligned, there is no need to sample th3
			if(th2 == 0.0) {
				goals.push_back(Vector3d(0.0, th2, th1));
				continue;
			}

			// Lastly, for any fixed location, we rotate the smallest joint
			for(int th3_idx = 0; th3_idx < num_th3; th3_idx++) {
				double th3 = (dth3 * th3_idx) + (turn3 ? 0.0 : M_2PI);
				goals.push_back(Vector3d(th1, th2, th3));
			}
		}
	}

	cout << "Generated " << goals.size() << " goals!\n";
//	for(int i = 0; i < goals.size(); i++) cout << goals[i].transpose() << endl;
}

/* ******************************************************************************************** */
void init () {

	// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
	// NOTE: We do not want to filter this data yet and we want it raw
	system("killall -s 9 netcanftd");
	usleep(20000);
	system("netcanftd -v -d -I lft -b 2 -B 1000 -c llwa_ft -r");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-gripperWeight";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the left arm
	initArm(daemon_cx, llwa, "llwa");
	somatic_motor_update(&daemon_cx, &llwa);

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open the state and ft channels 
	somatic_d_channel_open(&daemon_cx, &ft_chan, "llwa_ft", NULL);

	// Generate joint values to visit
	generateJointValues();
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}

