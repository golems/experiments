/**
 * @file 05-workspace.cpp
 * @author Saul Reynolds-Haertle, Can Erdogan, Munzir Zafar
 * @date July 25, 2013
 * @brief This file demonstrates workspace control running on
 * Krang. It will probably be a workhorse for general tasks, including
 * demonstrations.
 */

#include "kore.h"
#include "workspace.h"

#include <iostream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include "util.h"

using namespace Krang;

// initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const Eigen::VectorXd NULLSPACE_Q_REF_INIT = 
		(VectorXd(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.25;
const double SPACENAV_TRANSLATION_GAIN = 0.25; 
const double COMPLIANCE_GAIN = 1.0 / 750.0;

// various rate limits - be nice to other programs
const double LOOP_FREQUENCY = 300.0;

/* ********************************************************************************************* */
// State variables for the daemon - one daemon context and one

// hardware object
somatic_d_t daemon_cx;
ach_channel_t spacenav_chan;
Hardware* hw;

// pointers to important DART objects
simulation::World* world;
dynamics::SkeletonDynamics* robot;

// workspace stuff
WorkspaceControl* ws;
VectorXd nullspace_q_ref;

bool myDebug;

/* ******************************************************************************************** */
/// Clean up
void destroy() {

	// Stop the daemon
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
		SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Clean up the workspace stuff
	delete ws;

	// Close down the hardware
	delete hw;

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// Get the last time to compute the passed time
	int c_ = 0, badSpaceNavCtr = 0;
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	Eigen::VectorXd last_spacenav_input = Eigen::VectorXd::Zero(6);
	while(!somatic_sig_received) {

		myDebug = ((c_++ % 10) == 0);
		ws->debug = myDebug;
		if(myDebug) std::cout << "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << std::endl;

		// ============================================================================
		// Update state: get passed time and kinematics, sensor input and check for current limit

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// Update the robot
		hw->updateSensors(time_delta);
		if(myDebug) hw->printState();

		// Check for too high currents
		if(myDebug) DISPLAY_VECTOR(eig7(hw->larm->cur));
		if(checkCurrentLimits(eig7(hw->larm->cur))) {
			destroy();
			exit(EXIT_FAILURE);
		}

		// Update the spacenav taking care in the case of no inputs or spontaneous drops
		Eigen::VectorXd spacenav_input;
		if(!getSpaceNav(&spacenav_chan, spacenav_input)) 
			spacenav_input = (badSpaceNavCtr++ > 20) ? VectorXd::Zero(6) : last_spacenav_input;
		else badSpaceNavCtr = 0;
		last_spacenav_input = spacenav_input;

		// ============================================================================
		// Compute the desired jointspace velocity from the inputs

		// Compute qdot for our secondary goal (currently, drive qo to
		// a reference position
		Eigen::VectorXd q = robot->getConfig(*ws->arm_ids);
		Eigen::VectorXd qSecondary = q;
		qSecondary(3) = -0.5;
		Eigen::VectorXd qdot_secondary = qSecondary - q;

		// Compute the final xdot incorporating the input velocity and  compliance 
		Eigen::VectorXd qdot_jacobian;
		ws->update(spacenav_input, hw->lft->lastExternal, qdot_secondary, time_delta, qdot_jacobian);

		// ============================================================================
		// Threshold the input jointspace velocity for safety and send them

		// Scale the values down if the norm is too big or set it to zero if too small
		double magn = qdot_jacobian.norm();
		if(magn > 0.5) qdot_jacobian *= (0.5 / magn);
		else if(magn < 0.05) qdot_jacobian = VectorXd::Zero(7);
		if(myDebug) DISPLAY_VECTOR(qdot_jacobian);

		// Avoid joint limits - set velocities away from them as we get close
		Eigen::VectorXd qdot_avoid(7);
		if(myDebug) DISPLAY_VECTOR(q);
		computeQdotAvoidLimits(robot, *(ws->arm_ids), q, qdot_avoid);
		if(myDebug) DISPLAY_VECTOR(qdot_avoid);

		// Add our qdots together to get the overall movement
		Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;
		if(myDebug) DISPLAY_VECTOR(qdot_apply);

		// Send the velocity command
		somatic_motor_setvel(&daemon_cx, hw->larm, qdot_apply.data(), 7);

		// And sleep to fill out the loop period so we don't eat the entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		if(myDebug) std::cout << "sleeping for: " << time_sleep_usec << std::endl;
		usleep(time_sleep_usec);
	}
}

/* ******************************************************************************************** */
/// Initialization
void init() {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "05-workspace";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Hardware::Mode mode = (Hardware::Mode)((Hardware::MODE_ALL & ~Hardware::MODE_RARM) & ~Hardware::MODE_GRIPPERS);
	hw = new Hardware(mode, &daemon_cx, robot);

	// Open up the ach channel for the spacenav
	somatic_d_channel_open(&daemon_cx, &spacenav_chan, "spacenav-data", NULL);

	// Set up the workspace controllers
	ws = new WorkspaceControl(robot, LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
		SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN, COMPLIANCE_GAIN);
	nullspace_q_ref = VectorXd::Zero(7);

	// Start the daemon running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
		SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	init();
	run();
	destroy();
}
