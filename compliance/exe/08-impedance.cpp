/**
 * @file 08-impedance.cpp
 * @author Can Erdogan
 * @date June 23, 2013
 * @brief  
 */

#include "helpers.h"
#include <Eigen/StdVector>

using namespace std;

#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
	llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);

#define darm (cout << "dq: "<<llwa.vel[0] << ", " << llwa.vel[1] << ", " << llwa.vel[2] << ", " << \
	llwa.vel[3] << ", " << llwa.vel[4] << ", " << llwa.vel[5] << ", " << llwa.vel[6] << endl);

#define eig7(x) (Vector7d() << (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5], (x)[6]).finished()

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
somatic_motor_t llwa;

const bool recordTraj = 0;
vector <Vector7d, aligned_allocator<Vector7d> > traj;	///< The traj to follow in joint space pos

Vector7d accLimit = (Vector7d() << 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9).finished();
Vector7d velLimit = (Vector7d() << 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7).finished();

/* ********************************************************************************************* */
/// Limits the velocity using the last velocities and limits
void limit (Vector7d& vel, double dt) {

	// Determine the limits of velocities using the last velocity
	Vector7d velLowLim = (eig7(llwa.vel) - accLimit * dt).array().max(-velLimit.array());
	Vector7d velHighLim = (eig7(llwa.vel) + accLimit * dt).array().min(velLimit.array());
	
	// Limit the velocities with previous velocity and acceleration limits
	vel = vel.array().max(velLowLim.array());
	vel = vel.array().min(velHighLim.array());
}

/* ********************************************************************************************* */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// If we are going to follow a trajectory, it should not be empty
	assert((recordTraj || !traj.empty()) && "The trajectory should not be empty if no joystick.");

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0, traj_idx = 0;
	double dt = 1e-2;
	while(!somatic_sig_received) {

		c++;
		cout << "\nvvvvvvvvvvvvvvvvvvvvvvvvvv " << traj_idx << " vvvvvvvvvvvvvvvvvvvvvvvvvvvv" << endl;

		// If recording a trajectory, use joystick control and output the current position 
		if(recordTraj) {
			setJoystickInput(daemon_cx, js_chan, llwa, llwa);
			somatic_motor_update(&daemon_cx, &llwa);
			parm;
			usleep(1e7 * dt);
			continue;
		}

		if(traj_idx == traj.size() - 1) break;

		// Send the velocity commands 
		somatic_motor_cmd(&daemon_cx, &llwa, POSITION, traj[traj_idx].data(), 7, NULL);
		somatic_motor_update(&daemon_cx, &llwa);

		// Increment the position counter if we got close to our goal
		if((traj[traj_idx] - eig7(llwa.pos)).norm() < 1e-1) traj_idx++;

		usleep(1e6 * dt);
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
void init () {

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

	// Read the trajectory file
	double q [] = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0};	
	for(double i = M_PI_2; i >= -0.002; i -= 0.01) {
		q[4] = i;
		traj.push_back(eig7(q));	
	}
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}

