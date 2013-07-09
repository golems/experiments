/**
 * @file forwardKinematics.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date April 24, 2012
 * @brief This file receives the position of the red dot detected from the
 * vision PC and predicts the loaction of the red dot by forward kinematics
 * and then compares the two to determine the accuracy of the forward 
 * kinematics.
 */

#include <iostream>
#include "helpers.h"
#include "motion.h"

#define sq(x) ((x) * (x))

using namespace Eigen;
using namespace std;

/* ********************************************************************************************** */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t state_chan;
ach_channel_t chan_transform;
somatic_motor_t llwa, rlwa;

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	while(!somatic_sig_received) {

		// Read the joystick data and send the input velocities to the arms
		setJoystickInput(daemon_cx, js_chan, llwa, rlwa);

		// Receive the 3-D coordinate of the red dot from vision PC
		double x [3];
		bool gotKinect = getRedMarkerPosition(daemon_cx, chan_transform, &x[0]);
		if(!gotKinect) continue;
		
		// Get the right arm joint values and the end-effector value
		Vector3d pos, dir;
		somatic_motor_update(&daemon_cx, &llwa);
		getEEinKinectFrame(llwa.pos, pos, dir);					
		if(c++ % 10 == 0) {
			printf("Joint angles: ");
			for(size_t i = 0; i < 7; i++) 
				printf("%lf, ", llwa.pos[i]);
			printf("\n"); 
			cout << "Forward kinematics: " << pos.transpose() << endl;
			cout << "Kinect            :  " << x[0] << " " << x[1] << " " << x[2] << endl;
			double norm = sqrt(sq(pos[0]-x[0]) + sq(pos[1]-x[1]) + sq(pos[2]-x[2]));
			cout << "Norm              :  " << norm << "\n" << endl;
		}

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

	init(daemon_cx, llwa, rlwa, js_chan, state_chan, chan_transform);
	run();
	destroy();

	return 0;
}
