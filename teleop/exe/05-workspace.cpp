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
#include <ncurses.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include "util.h"

using namespace Krang;

/* ********************************************************************************************* */
// Constants

// initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.25;
const double SPACENAV_TRANSLATION_GAIN = 0.25; 
const double COMPLIANCE_GAIN = 1.0 / 750.0;

// various rate limits - be nice to other programs
const double LOOP_FREQUENCY = 300.0;
const double DISPLAY_FREQUENCY = 3.0;

// display information
const bool DO_CURSES = true;

/* ********************************************************************************************* */
// State variables

// hardware object
somatic_d_t daemon_cx;                          ///< daemon context
std::map<Krang::Side, Krang::SpaceNav*> spnavs; ///< points to spacenavs
std::map<Krang::Side, Krang::FT*> fts; ///< points to force-torque sensors

Hardware* hw;                                   ///< connects to hardware

// pointers to important DART objects
simulation::World* world;
dynamics::SkeletonDynamics* robot;

// workspace stuff
std::map<Krang::Side, Krang::WorkspaceControl*> wss; ///< does workspace control for the arms
std::map<Krang::Side, Vector7d> nullspace_qdot_refs; ///< nullspace configurations for the arms
Eigen::MatrixXd Trel_left_to_right; ///< translation from the left hand to the right hand
bool synch_mode;

// debug stuff
bool debug_print_this_it;       ///< whether we print

/* ******************************************************************************************** */
/// Curses initialization and cleanup
void init_curses() {
    if (!DO_CURSES) return;
    initscr();
    clear();
    noecho();                   // do not echo input to the screen
    cbreak();                   // do not buffer by line (receive characters immediately)
    timeout(0);                 // non-blocking getch
}

void destroy_curses() {
    if (!DO_CURSES) return;
    clrtoeol();
    refresh();
    endwin();
}

/* ******************************************************************************************** */
/// Clean up
void destroy() {
	// Stop the daemon
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
		SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Clean up the workspace stuff
	delete wss[Krang::LEFT];
	delete wss[Krang::RIGHT];

	// Close down the hardware
	delete spnavs[Krang::LEFT];
	delete spnavs[Krang::RIGHT];
	delete hw;

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	// close display
	destroy_curses();
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());

	while(!somatic_sig_received) {
        // get input from curses
        if(DO_CURSES) {
            int ch = getch();
            switch (ch) {
            case 'q': somatic_sig_received = true; break;
            case ' ':
	            synch_mode = !synch_mode;
	            if (synch_mode) {
		            Trel_left_to_right = wss[Krang::LEFT]->Tref.inverse() * wss[Krang::RIGHT]->Tref;
	            }
	            break;
            }
        }

		// ============================================================================
		// Update state: get passed time and kinematics, sensor input and check for current limit

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// set up debug printing
		debug_print_this_it = (time_now - time_last_display) > (1.0 / DISPLAY_FREQUENCY);

		// Update the robot
		hw->updateSensors(time_delta);

		// Check for too high currents
		if(checkCurrentLimits(eig7(hw->larm->cur))) {
			destroy();
			exit(EXIT_FAILURE);
		}

		if (!synch_mode) {
			// ================================================================================
			// Do workspace control for each arm individually
			for(int i = Krang::LEFT; i-1 < Krang::RIGHT; i++) {
				Krang::Side sde = static_cast<Krang::Side>(i);

				// Get the joint angles for the arm for later use
				Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

				// Construct a qdot that the jacobian will bias toward using the nullspace
				Eigen::VectorXd q_elbow_ref = q;
				q_elbow_ref(3) = sde == Krang::LEFT ? -0.5 : 0.5;
				nullspace_qdot_refs[sde] = (q_elbow_ref - q).normalized();

				// Get a workspace velocity input from the spacenav
				Eigen::VectorXd spacenav_input = spnavs[sde]->updateSpaceNav();
				Eigen::VectorXd xdot_spacenav = wss[sde]->uiInputVelToXdot(spacenav_input);

				// Compute the desired jointspace velocity from the inputs and sensors
				Eigen::VectorXd qdot_jacobian;
				wss[sde]->updateFromXdot(xdot_spacenav,
				                         fts[sde]->lastExternal,
				                         nullspace_qdot_refs[sde],
				                         time_delta,
				                         qdot_jacobian);
	
				// make sure we're not going too fast
				double magn = qdot_jacobian.norm();
				if (magn > 0.5) qdot_jacobian *= (0.5 / magn);
				if (magn < .05) qdot_jacobian *= 0.0;
	
				// avoid joint limits
				Eigen::VectorXd qdot_avoid(7);
				computeQdotAvoidLimits(robot, *wss[sde]->arm_ids, q, qdot_avoid);
	
				// add qdots together to get the overall movement
				Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;

				// // and apply that to the arm
				// somatic_motor_setvel(&daemon_cx,
				//                      sde == Krang::LEFT ? hw->larm : hw->rarm,
				//                      qdot_apply.data(),
				//                      7);
			}
		} else {
			// ================================================================================
			// Do workspace control for the left arm, then copy over a
			// position ref to the right arm
			
			// update nullspace bias
			for(int i = Krang::LEFT; i-1 < Krang::RIGHT; i++) {
				Krang::Side sde = static_cast<Krang::Side>(i);

				// Get the joint angles for the arm for later use
				Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

				// Construct a qdot that the jacobian will bias toward using the nullspace
				Eigen::VectorXd q_elbow_ref = q;
				q_elbow_ref(3) = sde == Krang::LEFT ? -0.5 : 0.5;
				nullspace_qdot_refs[sde] = (q_elbow_ref - q).normalized();
			}

			// ================================================================================
			// left arm

			// Get the joint angles for the arm for later use
			Eigen::VectorXd q_left = robot->getConfig(*wss[Krang::LEFT]->arm_ids);

			// Get a workspace velocity input from the spacenav
			Eigen::VectorXd spacenav_input_L = spnavs[Krang::LEFT]->updateSpaceNav();
			Eigen::VectorXd xdot_spacenav_L = wss[Krang::LEFT]->uiInputVelToXdot(spacenav_input_L);

			// compute the desired jointspace velocity from the inputs (ft is zero)
			Eigen::VectorXd qdot_jacobian_L;
			wss[Krang::LEFT]->updateFromXdot(xdot_spacenav_L,
			                                 fts[Krang::LEFT]->lastExternal,
			                                 nullspace_qdot_refs[Krang::LEFT],
			                                 time_delta,
			                                 qdot_jacobian_L);
	
			// make sure we're not going too fast
			double magn = qdot_jacobian_L.norm();
			if (magn > 0.5) qdot_jacobian_L *= (0.5 / magn);
			if (magn < .05) qdot_jacobian_L *= 0.0;
	
			// avoid joint limits
			Eigen::VectorXd qdot_avoid_L(7);
			computeQdotAvoidLimits(robot, *wss[Krang::LEFT]->arm_ids, q_left, qdot_avoid_L);
	
			// add qdots together to get the overall movement
			Eigen::VectorXd qdot_apply_L = qdot_avoid_L + qdot_jacobian_L;

			// // and apply that to the arm
			// somatic_motor_setvel(&daemon_cx, hw->larm, qdot_apply_L.data(), 7);

			// ================================================================================
			// right arm
	
			// figure out where the position reference should be using the
			// relative transofrmation from the left hand to the right hand
			Eigen::MatrixXd Tref_R = wss[Krang::LEFT]->Tref * Trel_left_to_right;

			// update the workspace controller for the other arm
			Eigen::VectorXd qdot_jacobian_R;
			wss[Krang::RIGHT]->updateFromUIPos(Tref_R,
			                                   fts[Krang::RIGHT]->lastExternal,
			                                   nullspace_qdot_refs[Krang::RIGHT],
			                                   qdot_jacobian_R);

			// make sure we're not going too fast
			magn = qdot_jacobian_R.norm();
			if (magn > 0.5) qdot_jacobian_R *= (0.5 / magn);
			if (magn < .05) qdot_jacobian_R *= 0.0;
	
			// avoid joint limits
			Eigen::VectorXd qdot_avoid_R(7);
			Eigen::VectorXd q_right = robot->getConfig(*wss[Krang::RIGHT]->arm_ids);
			computeQdotAvoidLimits(robot, *wss[Krang::RIGHT]->arm_ids, q_right, qdot_avoid_R);

			// add qdots together to get the overall movement
			Eigen::VectorXd qdot_apply_R = qdot_avoid_R + qdot_jacobian_R;
	
			// // and apply that to the right arm
			// somatic_motor_setvel(&daemon_cx, hw->rarm, qdot_apply_R.data(), 7);
		}
		
		if (debug_print_this_it) {
			mvprintw(1, 1, "synch mode: %d", synch_mode);
		}
		
		// And sleep to fill out the loop period so we don't eat the entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		usleep(time_sleep_usec);
	}
}

/* ******************************************************************************************** */
/// Initialization
void init() {
	// initialize display
	init_curses();

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

	// fill out the convenient force-torque pointers
	fts[Krang::LEFT] = hw->lft;
	fts[Krang::RIGHT] = hw->rft;

	// Set up the workspace stuff
	synch_mode = false;
	wss[Krang::LEFT] = new WorkspaceControl(robot, LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
	                                        SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN, COMPLIANCE_GAIN);
	wss[Krang::RIGHT] = new WorkspaceControl(robot, RIGHT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
	                                         SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN, COMPLIANCE_GAIN);

	// Initialize the spacenavs
	spnavs[Krang::LEFT] = new Krang::SpaceNav(&daemon_cx, "spacenav-data-l", .5);
	spnavs[Krang::RIGHT] = new Krang::SpaceNav(&daemon_cx, "spacenav-data-r", .5);

	// set up the relative transform between the hands
	Trel_left_to_right = wss[Krang::LEFT]->Tref.inverse() * wss[Krang::RIGHT]->Tref;

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
