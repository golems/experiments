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
#include "display.hpp"

#include <iostream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include "util.h"

using namespace Krang;

/* ********************************************************************************************* */
// Declarations
typedef enum {
	MANIP_MODE_OFF,
	MANIP_MODE_SPNAV,
	MANIP_MODE_SPNAV_NOROT,
	MANIP_MODE_SYNCH,
	MANIP_MODE_HAND_OVER_HAND,
} manip_mode_t;

typedef enum {
	MANIP_PRIMARY_LEFT = Krang::LEFT,
	MANIP_PRIMARY_RIGHT = Krang::RIGHT,
	MANIP_BOTH,
} manip_primary_t;

/* ********************************************************************************************* */
// Constants

// initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.50; // maximum 50 cm per second from spacenav
const double SPACENAV_TRANSLATION_GAIN = 0.25; // maximum .25 radians per second from spacenav
const double COMPLIANCE_TRANSLATION_GAIN = 1.0 / 750.0;
const double COMPLIANCE_ORIENTATION_GAIN = .125 / 750.0;
const double HAND_OVER_HAND_SPEED = 0.03; // 3 cm per second when going hand-over-hand

// various rate limits - be nice to other programs
const double LOOP_FREQUENCY = 15.0;
const double DISPLAY_FREQUENCY = 3.0;

// display information
const int CURSES_DEBUG_DISPLAY_START = 20;

// gripper reference messages, so we don't have to do initialization in the middle of our program
double GRIPPER_CURRENT_OPEN[] = {10.0};
double GRIPPER_CURRENT_CLOSE[] = {-10.0};
double GRIPPER_CURRENT_ZERO[] = {0.0};

double GRIPPER_POSITION_OPEN[] = {0, 0, 0, 128};
double GRIPPER_POSITION_CLOSE[] = {255, 255, 255, 128};
double GRIPPER_POSITION_PARTIAL[] = {70, 70, 70, 128};

/* ********************************************************************************************* */
// State variables

// hardware object
somatic_d_t daemon_cx;                          ///< daemon context
std::map<Krang::Side, Krang::SpaceNav*> spnavs; ///< points to spacenavs
std::map<Krang::Side, Krang::FT*> fts; ///< points to force-torque sensors

Hardware* hw;                                   ///< connects to hardware
bool sending_commands = false;

// pointers to important DART objects
simulation::World* world;
dynamics::SkeletonDynamics* robot;

// workspace stuff
manip_mode_t ws_mode;
manip_primary_t ws_primary;
std::map<Krang::Side, Krang::WorkspaceControl*> wss; ///< does workspace control for the arms
std::map<Krang::Side, Vector7d> nullspace_qdot_refs; ///< nullspace configurations for the arms
Eigen::MatrixXd Trel_left_to_right; ///< translation from the left hand to the right hand
bool synch_mode;
bool fixed_orientation_mode;

// hand-over-hand stuff
std::map<Krang::Side, Vector3d> hoh_initpos;

// debug stuff
bool debug_print_this_it;       ///< whether we print
int Krang::curses_display_row = 30;
int Krang::curses_display_precision = 15;
bool Krang::doing_curses = false;
// std::ofstream debug_log;
double time_log_start;

int Krang::COLOR_RED_BACKGROUND = 11;
int Krang::COLOR_YELLOW_BACKGROUND = 12;
int Krang::COLOR_GREEN_BACKGROUND = 13;
int Krang::COLOR_WHITE_BACKGROUND = 14;

/* ******************************************************************************************** */
/// Clean up
void destroy() {

	// close display
	Krang::destroy_curses();

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

}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());

	// initialize display
	Krang::init_curses();

	while(!somatic_sig_received) {

		// ========================================================================================
		// Handle input from the user via curses
		int ch = getch();
		switch (ch) {
		case 'q': somatic_sig_received = true; break;
		case 'r': {
			somatic_motor_reset(&daemon_cx, hw->larm);
			somatic_motor_reset(&daemon_cx, hw->rarm);
		} break;
		case 'h': {
			ws_mode = MANIP_MODE_OFF;
			sending_commands = !sending_commands;
			somatic_motor_halt(&daemon_cx, hw->larm);
			somatic_motor_halt(&daemon_cx, hw->rarm);
		} break;
		case 'u': {
			somatic_motor_setpos(&daemon_cx, hw->lgripper, GRIPPER_POSITION_PARTIAL, 4);
		} break;
		case 'i': {
			somatic_motor_setpos(&daemon_cx, hw->rgripper, GRIPPER_POSITION_PARTIAL, 4);
		} break;
		case ' ': {
			// TODO: halt or unhalt motors as necessary
			sending_commands = !sending_commands;
			if (sending_commands) {
				somatic_motor_reset(&daemon_cx, hw->larm);
				somatic_motor_reset(&daemon_cx, hw->rarm);
				wss[Krang::LEFT]->resetReferenceTransform();
				wss[Krang::RIGHT]->resetReferenceTransform();
			} else {
				Eigen::VectorXd z = Eigen::VectorXd::Zero(7);
				somatic_motor_setvel(&daemon_cx, hw->larm, z.data(), 7);
				somatic_motor_setvel(&daemon_cx, hw->rarm, z.data(), 7);
			}
		} break;
		case 's': {
			synch_mode = !synch_mode;
			if (synch_mode) {
				Trel_left_to_right = wss[Krang::LEFT]->Tref.inverse() * wss[Krang::RIGHT]->Tref;
			}
		} break;
		case '8':
			// begin left hand hand-over-hand
			break;
		}
		case '7':
			// left hand go hand-over-hand away from right hand
			break;
		}
		case '9':
			// left hand go hand-over-hand away from right hand
			break;
		}
		case '5':
			// begin right hand hand-over-hand
			break;
		}
		case '4':
			// right hand go hand-over-hand away from left hand
			break;
		}
		case '6':
			// right hand go hand-over-hand away from left hand
			break;
		}
		Krang::curses_display_row = CURSES_DEBUG_DISPLAY_START;

		// ========================================================================================
		// Update state: get passed time and kinematics, sensor input and check for current limit

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// set up debug printing
		debug_print_this_it = (time_now - time_last_display) > (1.0 / DISPLAY_FREQUENCY);
		if(debug_print_this_it) time_last_display = time_now;
		wss[Krang::LEFT]->debug_to_curses = debug_print_this_it;
		wss[Krang::RIGHT]->debug_to_curses = debug_print_this_it;

		// print some general debug output
		if (debug_print_this_it) {
			hw->printStateCurses(5, 1);
			attron(COLOR_PAIR(sending_commands?COLOR_RED:COLOR_WHITE));
			mvprintw(11, 1, "sendcmds mode: %d", sending_commands);
			attroff(COLOR_PAIR(sending_commands?COLOR_RED:COLOR_WHITE));
		}
		if (debug_print_this_it) { mvprintw(12, 1, "synch mode: %d", synch_mode); }

		// Update the robot
		hw->updateSensors(time_delta);

		// Check for too high currents
		if(checkCurrentLimits(eig7(hw->larm->cur)) && checkCurrentLimits(eig7(hw->rarm->cur))) {
			if (sending_commands) {
				sending_commands = false;
				somatic_motor_halt(&daemon_cx, hw->larm);
				somatic_motor_halt(&daemon_cx, hw->rarm);
			}

			// // TODO: handle this more nicely
			// destroy();
			// exit(EXIT_FAILURE);
		}

		// ========================================================================================
		// Perform workspace for each arm, changing the input for right arm based on synch mode

		Eigen::MatrixXd Tref_R_sync;
		Eigen::VectorXd spacenav_input, xdot_spacenav;
		for(int sint = Krang::LEFT; sint-1 < Krang::RIGHT; sint++) {
			// Get the arm side and the joint angles
			Krang::Side sde = static_cast<Krang::Side>(sint);
			Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

			if (debug_print_this_it) {
				Krang::curses_display_row++;
				mvprintw(Krang::curses_display_row++, 1, "Arm: '%s'\n", sde == Krang::LEFT ? "LEFT" : "RIGHT");
			}

			// Nullspace: construct a qdot that the jacobian will bias toward using the nullspace
			Eigen::VectorXd q_elbow_ref = q;
			q_elbow_ref(3) = sde == Krang::LEFT ? -0.5 : 0.5;
			nullspace_qdot_refs[sde] = (q_elbow_ref - q).normalized();

			// update spacenav, because we use it for grippers regardless of the mode
			spacenav_input = spnavs[sde]->updateSpaceNav();

			// depending on the synch mode, get a workspace velocity either from the spacenav or
			// from the other arm
			if(synch_mode && (sde == Krang::RIGHT)) {
				Tref_R_sync = wss[Krang::LEFT]->Tref * Trel_left_to_right;
			} else {
				xdot_spacenav = wss[sde]->uiInputVelToXdot(spacenav_input);
			}

			// Close the gripper if button 0 is pressed, open it if button 1.
			somatic_motor_t* gripper = (sde == Krang::LEFT) ? hw->lgripper : hw->rgripper;
			if(spnavs[sde]->buttons[sint] == 1) {
				// somatic_motor_reset(&daemon_cx, gripper);
				// usleep(100);
				// somatic_motor_cmd(&daemon_cx, gripper, 
				//                   SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, GRIPPER_CURRENT_CLOSE, 1, NULL);
				somatic_motor_setpos(&daemon_cx, gripper, GRIPPER_POSITION_OPEN, 4);
			}
			if(spnavs[sde]->buttons[(sint + 1) % 2] == 1) {
				// somatic_motor_reset(&daemon_cx, gripper);
				// usleep(100);
				// somatic_motor_cmd(&daemon_cx, gripper, 
				//                   SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, GRIPPER_CURRENT_OPEN, 1, NULL);
				somatic_motor_setpos(&daemon_cx, gripper, GRIPPER_POSITION_CLOSE, 4);
			}

			// Jacobian: compute the desired jointspace velocity from the inputs and sensors
			Eigen::VectorXd qdot_jacobian;
			if(synch_mode && (sde == Krang::RIGHT)) {
				wss[Krang::RIGHT]->updateFromUIPos(Tref_R_sync, fts[Krang::RIGHT]->lastExternal,
				                                   nullspace_qdot_refs[Krang::RIGHT], qdot_jacobian);
			}
			else {
				wss[sde]->updateFromXdot(xdot_spacenav, fts[sde]->lastExternal,
				                         nullspace_qdot_refs[sde], time_delta, qdot_jacobian);
			}

			// make sure we're not going too fast
			double magn = qdot_jacobian.norm();
			if (magn > 0.5) qdot_jacobian *= (0.5 / magn);
			if (magn < .05) qdot_jacobian *= 0.0;

			// avoid joint limits
			Eigen::VectorXd qdot_avoid(7);
			computeQdotAvoidLimits(robot, *wss[sde]->arm_ids, q, qdot_avoid);

			// add qdots together to get the overall movement
			Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;

			// and apply that to the arm
			if(sending_commands)
				somatic_motor_setvel(&daemon_cx, sde == Krang::LEFT ? hw->larm : hw->rarm, qdot_apply.data(), 7);

			if (debug_print_this_it) {
				// if(synch_mode && (sde == Krang::RIGHT)) 
				// 	Krang::curses_display_matrix(Tref_R_sync);
				Eigen::MatrixXd ee_trans = wss[sde]->endEffector->getWorldTransform();
				Eigen::VectorXd ee_pos = Krang::transformToEuler(ee_trans, math::XYZ);

				Krang::curses_display_vector(ee_pos, "current ee pos");
				Krang::curses_display_vector(fts[sde]->lastExternal, "ft");
				Krang::curses_display_vector(nullspace_qdot_refs[sde], "nullspace_qdot");
				Krang::curses_display_vector(xdot_spacenav, "xdot_spacenav");
				Krang::curses_display_vector(qdot_jacobian, "qdot_jacobian");
				Krang::curses_display_vector(qdot_apply, "qdot_apply");

				Eigen::VectorXd cur(7);
				double largest_cur = 0;
				for (int i = 0; i < 7; i++) {
					cur[i] = (sde == Krang::LEFT ? hw->larm->cur[i] : hw->rarm->cur[i]);
					largest_cur = std::max(largest_cur, cur[i]);
				}
				Krang::curses_display_vector(cur, "measured_current", 0, largest_cur < 8 ? COLOR_WHITE : COLOR_YELLOW);
			}
		}

		// display everything curses has done
		refresh();

		// And sleep to fill out the loop period so we don't eat the entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		if (debug_print_this_it) {
			Krang::curses_display_row++;
			mvprintw(Krang::curses_display_row++, 1, "Loop period: %f/%f seconds", time_delta, (1.0/LOOP_FREQUENCY));
			mvprintw(Krang::curses_display_row++, 1, "Will sleep for %f seconds", time_sleep);
		}
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
	Hardware::Mode mode = (Hardware::Mode)(Hardware::MODE_ALL);
	hw = new Hardware(mode, &daemon_cx, robot);

	// fill out the convenient force-torque pointers
	fts[Krang::LEFT] = hw->lft;
	fts[Krang::RIGHT] = hw->rft;

	// Set up the workspace stuff
	synch_mode = false;
	fixed_orientation_mode = false;
	wss[Krang::LEFT] = new WorkspaceControl(robot, LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
	                                        SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN, COMPLIANCE_TRANSLATION_GAIN,
	                                        COMPLIANCE_ORIENTATION_GAIN);
	wss[Krang::RIGHT] = new WorkspaceControl(robot, RIGHT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
	                                         SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN, COMPLIANCE_TRANSLATION_GAIN,
	                                         COMPLIANCE_ORIENTATION_GAIN);

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
