/**
 * @file 01- teleop_joystick.cpp
 * @author Jonathan Scholz
 * @date July 7, 2013
 * @briefs This executable demonstrates the joint space control of the arms with the joystick
 * input.
 */

#include <iostream>
#include <Eigen/Dense>

#define sq(x) ((x) * (x))

using namespace std;
using namespace Eigen;


/* ********************************************************************************************** */
/*
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t state_chan;
ach_channel_t chan_transform;
somatic_motor_t llwa, rlwa;
*/
/* ********************************************************************************************* */
/*
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
*/
/* ********************************************************************************************* */
/*
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);

	somatic_d_channel_close(&daemon_cx, &chan_transform);
	somatic_d_destroy(&daemon_cx);
}
*/
/* ********************************************************************************************* */
/*
void init () {

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
}
*/
/* ********************************************************************************************* */
int main() {
  /*
	init(daemon_cx, llwa, rlwa, js_chan, state_chan, chan_transform);
	run();
	destroy();
  */
	return 0;
}
