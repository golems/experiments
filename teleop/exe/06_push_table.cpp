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
 * @file 05-workspace.cpp
 * @author Saul Reynolds-Haertle, Can Erdogan, Munzir Zafar
 * @date July 25, 2013
 * @brief This file demonstrates workspace control running on
 * Krang. It will probably be a workhorse for general tasks, including
 * demonstrations.
 */

#include <argp.h>

#include <kore.hpp>
#include <kore/workspace.hpp>
#include <kore/display.hpp>
#include <kore/util.hpp>

#include <iostream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <somatic/msg.h>



// context struct
typedef struct {
	somatic_d_t d;
	somatic_d_opts_t d_opts;
	int opt_home_mode;
} cx_t;

/* ----------------- */
/* ARGP Definitions  */
/* ----------------- */
static struct argp_option options[] = {
		{"home_mode", 'h', NULL, 0, "home arm rather than pushing"},
};

/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "push_table-0.0.1";
/// argp program arguments documentation
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "hack: pushes a table";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };

static int parse_opt( int key, char *arg, struct argp_state *state) {
	cx_t *cx = (cx_t*)state->input;
	switch(key) {
	case 'h':
		cx->opt_home_mode++;
		break;
	case 0:
		break;
	}

	somatic_d_argp_parse( key, arg, &cx->d_opts );

	return 0;
}

/* ********************************************************************************************* */
// Type declarations

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
const double HAND_OVER_HAND_SPEED = 0.05; // 3 cm per second when going hand-over-hand

// various rate limits - be nice to other programs
const double LOOP_FREQUENCY = 15.0;
const double DISPLAY_FREQUENCY = 3.0;

// display information
const int CURSES_DEBUG_DISPLAY_START = 20;

// gripper reference messages, so we don't have to do initialization in the middle of our program
double GRIPPER_CURRENT_OPEN[] = {10.0};
double GRIPPER_CURRENT_CLOSE[] = {-10.0};
double GRIPPER_CURRENT_ZERO[] = {0.0};

// Gripper constants (could move to constructor, but convenient here)
double ROBOTIQ_GRIPPER_POSITION_OPEN[] = {0, 0, 0, 128};
double ROBOTIQ_GRIPPER_POSITION_CLOSE[] = {255, 255, 255, 128};
double ROBOTIQ_GRIPPER_POSITION_PARTIAL[] = {70, 70, 70, 128};
double SCHUNK_GRIPPER_POSITION_OPEN[] = {0.9};
double SCHUNK_GRIPPER_POSITION_CLOSE[] = {0};
double SCHUNK_GRIPPER_POSITION_PARTIAL[] = {0.4};

/* ********************************************************************************************* */
// State variables

// hardware objects
Krang::Hardware* hw;                                   ///< connects to hardware
bool sending_commands = true;
std::map<Krang::Side, Krang::SpaceNav*> spnavs; ///< points to spacenavs

// process state
somatic_d_t daemon_cx;                          ///< daemon context
Somatic__VisualizeData* vis_msg;

// pointers to important DART objects
simulation::World* world;
dynamics::SkeletonDynamics* robot;

// workspace stuff
std::map<Krang::Side, Krang::WorkspaceControl*> wss; ///< does workspace control for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_q_refs; ///< nullspace configurations for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_q_masks; ///< nullspace configurations for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_qdot_refs; ///< nullspace configurations for the arms
Eigen::MatrixXd Trel_pri_to_off; ///< translation from the primary hand to the off hand

// mode stuff
bool synch_mode = false;
bool hoh_mode = false;
Krang::Side primary_hand = Krang::LEFT;
Krang::Side off_hand = Krang::RIGHT;
bool hoh_moving_right = false;
bool spacenavs_flipped = false;

// hand-over-hand stuff
std::map<Krang::Side, Eigen::Vector3d> hoh_initpos;

// debug information
bool debug_print_this_it;       ///< whether we print

// stuff for visualization
ach_channel_t vis_chan; // saul's whatever thing
ach_channel_t vision_chan; // vision channel for reading tag info (need in sync w/ control stuff)
size_t frame_size = 512;
size_t frame_cnt = 10;
double time_start;

/* ******************************************************************************************** */
/// Clean up
void destroy() {

	// close display
	Krang::destroy_curses();

	// Stop the daemon
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// close the channel we use for publishing visualization data
	somatic_d_channel_close(&daemon_cx, &vis_chan);

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
			somatic_motor_reset(&daemon_cx, hw->arms[Krang::LEFT]);
			somatic_motor_reset(&daemon_cx, hw->arms[Krang::RIGHT]);
		} break;
		case 'h': {
			hoh_mode = false;
			sending_commands = false;
			somatic_motor_halt(&daemon_cx, hw->arms[Krang::LEFT]);
			somatic_motor_halt(&daemon_cx, hw->arms[Krang::RIGHT]);
		} break;
		case 'u': {
			if (hw->mode & Krang::Hardware::MODE_GRIPPERS_SCH)
				somatic_motor_setpos(&daemon_cx, hw->grippers[Krang::LEFT], SCHUNK_GRIPPER_POSITION_PARTIAL, 1);
			else if (hw->mode & Krang::Hardware::MODE_GRIPPERS)
				somatic_motor_setpos(&daemon_cx, hw->grippers[Krang::LEFT], ROBOTIQ_GRIPPER_POSITION_PARTIAL, 4);
		} break;
		case 'i': {
			if (hw->mode & Krang::Hardware::MODE_GRIPPERS_SCH)
				somatic_motor_setpos(&daemon_cx, hw->grippers[Krang::RIGHT], SCHUNK_GRIPPER_POSITION_PARTIAL, 1);
			else if (hw->mode & Krang::Hardware::MODE_GRIPPERS)
				somatic_motor_setpos(&daemon_cx, hw->grippers[Krang::RIGHT], ROBOTIQ_GRIPPER_POSITION_PARTIAL, 4);
		} break;
		case 'f': {
			spacenavs_flipped = !spacenavs_flipped;
			hoh_mode = false;
		} break;
		case ' ': {
			sending_commands = !sending_commands;
			hoh_mode = false;
			wss[Krang::LEFT]->resetReferenceTransform();
			wss[Krang::RIGHT]->resetReferenceTransform();
			Trel_pri_to_off = wss[primary_hand]->Tref.inverse() * wss[off_hand]->Tref;
			if (sending_commands) {
				somatic_motor_reset(&daemon_cx, hw->arms[Krang::LEFT]);
				somatic_motor_reset(&daemon_cx, hw->arms[Krang::RIGHT]);
			} else {
				Eigen::VectorXd z = Eigen::VectorXd::Zero(7);
				somatic_motor_setvel(&daemon_cx, hw->arms[Krang::LEFT], z.data(), 7);
				somatic_motor_setvel(&daemon_cx, hw->arms[Krang::RIGHT], z.data(), 7);
			}
		} break;
		case 's': {
			synch_mode = !synch_mode;
			Trel_pri_to_off = wss[primary_hand]->Tref.inverse() * wss[off_hand]->Tref;
		} break;
		case '5': {
			// start or stop moving hand-over-hand
			hoh_mode = !hoh_mode;
			hoh_initpos[Krang::RIGHT] = wss[Krang::RIGHT]->endEffector->getWorldTransform().topRightCorner<3,1>();
			hoh_initpos[Krang::LEFT] = wss[Krang::LEFT]->endEffector->getWorldTransform().topRightCorner<3,1>();
		} break;
		case '1': {
			// select moving leftward hand-over-hand
			hoh_moving_right = false;
		} break;
		case '3': {
			// select moving to the right hand-over-hand
			hoh_moving_right = true;
		} break;
		case '4': {
			// select the left hand for hand-over-hand movement
			primary_hand = Krang::LEFT;
			off_hand = Krang::RIGHT;
			Trel_pri_to_off = wss[primary_hand]->Tref.inverse() * wss[off_hand]->Tref;
		} break;
		case '6': {
			// select the right hand for hand-over-hand movement
			primary_hand = Krang::RIGHT;
			off_hand = Krang::LEFT;
			Trel_pri_to_off = wss[primary_hand]->Tref.inverse() * wss[off_hand]->Tref;
		} break;
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
			hw->printStateCurses(3, 1);

			attron(COLOR_PAIR(sending_commands?Krang::COLOR_RED_BACKGROUND:COLOR_WHITE));
			mvprintw(11, 1, "sendcmds mode: %s", sending_commands?"yes":"no ");
			attroff(COLOR_PAIR(sending_commands?Krang::COLOR_RED_BACKGROUND:COLOR_WHITE));

			attron(COLOR_PAIR(synch_mode?Krang::COLOR_RED_BACKGROUND:COLOR_WHITE));
			mvprintw(12, 1, "synch mode: %s", synch_mode?"yes":"no ");
			attroff(COLOR_PAIR(synch_mode?Krang::COLOR_RED_BACKGROUND:COLOR_WHITE));

			attron(COLOR_PAIR(hoh_mode?Krang::COLOR_RED_BACKGROUND:COLOR_WHITE));
			mvprintw(13, 1, "hand-over-hand mode: %s", hoh_mode?"yes":"no ");
			attroff(COLOR_PAIR(hoh_mode?Krang::COLOR_RED_BACKGROUND:COLOR_WHITE));

			mvprintw(14, 5, "hoh side: ");
			if (primary_hand == Krang::LEFT) mvprintw(14, 16, "left       ");
			else mvprintw(14, 16, "      right");
			mvprintw(15, 5, " hoh dir: ");
			if (hoh_moving_right) mvprintw(15, 16, "      right");
			else mvprintw(15, 16, "left       ");


			attron(COLOR_PAIR(spacenavs_flipped?Krang::COLOR_RED_BACKGROUND:COLOR_WHITE));
			mvprintw(17, 1, "Input flipped: %s", spacenavs_flipped?"yes":"no ");
			attroff(COLOR_PAIR(spacenavs_flipped?Krang::COLOR_RED_BACKGROUND:COLOR_WHITE));
		}

		// Update the robot
		hw->updateSensors(time_delta);

		// Check for too high currents
		if(Krang::checkCurrentLimits(hw->arms[Krang::LEFT]->cur, 7)
		   && Krang::checkCurrentLimits(hw->arms[Krang::RIGHT]->cur, 7)) {
			if (sending_commands) {
				sending_commands = false;
				hoh_mode = false;
				somatic_motor_halt(&daemon_cx, hw->arms[Krang::LEFT]);
				somatic_motor_halt(&daemon_cx, hw->arms[Krang::RIGHT]);
			}

			// // TODO: handle this more nicely
			// destroy();
			// exit(EXIT_FAILURE);
		}

		// ========================================================================================
		// Perform workspace for each arm, changing the input for right arm based on synch mode

		for(int sint = Krang::LEFT; sint-1 < Krang::RIGHT; sint++) {
			// Get the arm side and the joint angles
			Krang::Side sde = static_cast<Krang::Side>(sint);
			Krang::Side sde_other = (sde==Krang::LEFT) ? Krang::RIGHT : Krang::LEFT;
			Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

			// some debugs
			if (debug_print_this_it) {
				// clear out a row to separate the arms' prints
				mvprintw(Krang::curses_display_row, 1, "                                                                                                                        ");
				Krang::curses_display_row++;
				mvprintw(Krang::curses_display_row++, 1, "Arm: '%s'\n", sde == Krang::LEFT ? "LEFT" : "RIGHT");
			}

			// Create variables
			Eigen::MatrixXd Tref_off_sync;
			Eigen::VectorXd spacenav_input;
			Eigen::VectorXd xdot_spacenav = Eigen::VectorXd::Zero(6);
			Eigen::VectorXd xdot_hoh = Eigen::VectorXd::Zero(6);
			Eigen::VectorXd xdot_ws_goal = Eigen::VectorXd::Zero(6);

			// Nullspace: construct a qdot that the jacobian will bias toward using the nullspace
			nullspace_qdot_refs[sde] = (nullspace_q_refs[sde] - q).cwiseProduct(nullspace_q_masks[sde]);

			// update spacenav, because we use it for grippers regardless of the mode
			if (spacenavs_flipped)
				spacenav_input = spnavs[sde_other]->updateSpaceNav();
			else
				spacenav_input = spnavs[sde]->updateSpaceNav();

			// Close the gripper if button 0 is pressed, open it if button 1.
			if(spnavs[sde]->buttons[sint] == 1) {
				if (hw->mode & Krang::Hardware::MODE_GRIPPERS_SCH) {
					somatic_motor_reset(&daemon_cx, hw->grippers[sde]);
					usleep(1e4);
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], SCHUNK_GRIPPER_POSITION_OPEN, 1);
				}
				else if (hw->mode & Krang::Hardware::MODE_GRIPPERS)
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], ROBOTIQ_GRIPPER_POSITION_OPEN, 4);
			}
			if(spnavs[sde]->buttons[(sint + 1) % 2] == 1) {
				if (hw->mode & Krang::Hardware::MODE_GRIPPERS_SCH) {
					somatic_motor_reset(&daemon_cx, hw->grippers[sde]);
					usleep(1e4);
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], SCHUNK_GRIPPER_POSITION_CLOSE, 1);
				}
				else if (hw->mode & Krang::Hardware::MODE_GRIPPERS)
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], ROBOTIQ_GRIPPER_POSITION_CLOSE, 4);

			}

			// depending on the synch mode, get a workspace velocity either from the spacenav or
			// from the other arm
			if(synch_mode && sde == off_hand) {
				Tref_off_sync = wss[primary_hand]->Tref * Trel_pri_to_off;
			} else if (!hoh_mode) {
				xdot_spacenav = wss[sde]->uiInputVelToXdot(spacenav_input);
			} else if (hoh_mode && sde == primary_hand) {
				Eigen::Vector3d dx_hoh = hoh_initpos[Krang::RIGHT] - hoh_initpos[Krang::LEFT];
				if (!hoh_moving_right)
					dx_hoh *= -1;
				xdot_hoh = Eigen::VectorXd::Zero(6);
				xdot_hoh.topLeftCorner<3,1>() = dx_hoh.normalized() * HAND_OVER_HAND_SPEED;
			}

			// put together the inputs from spacenav and hand-over-hand
			xdot_ws_goal = xdot_spacenav + xdot_hoh;

			// Jacobian: compute the desired jointspace velocity from the inputs and sensors
			Eigen::VectorXd qdot_jacobian;
			if(synch_mode && sde == off_hand) {
				wss[sde]->updateFromUIPos(Tref_off_sync, hw->fts[sde]->lastExternal,
				                          nullspace_qdot_refs[sde], qdot_jacobian);
			}
			else {
				wss[sde]->updateFromXdot(xdot_ws_goal, hw->fts[sde]->lastExternal,
				                         nullspace_qdot_refs[sde], time_delta, qdot_jacobian);
			}

			// make sure we're not going too fast
			double magn = qdot_jacobian.norm();
			if (magn > 0.5) qdot_jacobian *= (0.5 / magn);
			if (magn < .05) qdot_jacobian *= 0.0;

			// avoid joint limits
			Eigen::VectorXd qdot_avoid(7);
			Krang::computeQdotAvoidLimits(robot, *wss[sde]->arm_ids, q, qdot_avoid);

			// add qdots together to get the overall movement
			Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;

			// and apply that to the arm
			if(sending_commands)
				somatic_motor_setvel(&daemon_cx, hw->arms[sde], qdot_apply.data(), 7);

			// store some internal state into the vis_msg so we can publish it
			Eigen::VectorXd posref_euler = Krang::transformToEuler(wss[sde]->Tref, math::XYZ);
			for(int i = 0; i < 6; i++) vis_msg->vecs[0 + (4*sint)]->data[i] = posref_euler[i];
			for(int i = 0; i < 6; i++) vis_msg->vecs[1 + (4*sint)]->data[i] = xdot_spacenav[i];
			for(int i = 0; i < 6; i++) vis_msg->vecs[2 + (4*sint)]->data[i] = xdot_hoh[i];
			for(int i = 0; i < 6; i++) vis_msg->vecs[3 + (4*sint)]->data[i] = hw->fts[sde]->lastExternal[i];

			// do some debug output by printing to curses
			if (debug_print_this_it) {
				// if(synch_mode && (sde == Krang::RIGHT))
				// 	Krang::curses_display_matrix(Tref_R_sync);
				Eigen::MatrixXd ee_trans = wss[sde]->endEffector->getWorldTransform();
				Eigen::VectorXd ee_pos = Krang::transformToEuler(ee_trans, math::XYZ);

				Krang::curses_display_vector(hw->fts[sde]->lastExternal,   "ft                                  ");
				Krang::curses_display_vector(xdot_spacenav,            "xdot_spacenav                       ");
				Krang::curses_display_vector(xdot_hoh,                 "xdot_hoh                            ");
				Krang::curses_display_vector(xdot_ws_goal,             "xdot_ws_goal                        ");
				Krang::curses_display_vector(nullspace_qdot_refs[sde], "nullspace_qdot                      ");
				Krang::curses_display_vector(qdot_jacobian,            "qdot_jacobian                       ");
				Krang::curses_display_vector(qdot_apply,               "qdot_apply                          ");
				Krang::curses_display_vector(ee_pos,                   "current ee pos                      ");

				Eigen::VectorXd cur(7);
				double largest_cur = 0;
				for (int i = 0; i < 7; i++) {
					cur[i] = hw->arms[sde]->cur[i];
					largest_cur = std::max(largest_cur, cur[i]);
				}

				if(largest_cur < .1) {
					attron(COLOR_PAIR(Krang::COLOR_RED_BACKGROUND));
					mvprintw(11, sde==Krang::LEFT ? 30 : 50,
					         sde==Krang::LEFT ? "left halt?" : "right halt?");
					attroff(COLOR_PAIR(Krang::COLOR_RED_BACKGROUND));
				} else {
					mvprintw(11, sde==Krang::LEFT ? 30 : 50,
					         sde==Krang::LEFT ? "          " : "           ");
				}

				Krang::curses_display_vector(cur, "measured_current                    ",
				                             0, largest_cur < 8 ? COLOR_WHITE : COLOR_YELLOW);
			}

			// publish our visualization data
			SOMATIC_PACK_SEND(&vis_chan, somatic__visualize_data, vis_msg);
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
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "06-push_table";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware for the appropriate gripper mode
	Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL_GRIPSCH);
	hw = new Krang::Hardware(mode, &daemon_cx, robot);

	// Initialize the spacenavs
	spnavs[Krang::LEFT] = new Krang::SpaceNav(&daemon_cx, "joystick-data", .5); // spacenav-data-l
	spnavs[Krang::RIGHT] = new Krang::SpaceNav(&daemon_cx, "spacenav-data-r", .5);

	// Set up the workspace stuff
	wss[Krang::LEFT] = new Krang::WorkspaceControl(robot, Krang::LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN,
	                                               SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN,
	                                               COMPLIANCE_TRANSLATION_GAIN, COMPLIANCE_ORIENTATION_GAIN);
	wss[Krang::RIGHT] = new Krang::WorkspaceControl(robot, Krang::RIGHT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN,
	                                                SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN,
	                                                COMPLIANCE_TRANSLATION_GAIN, COMPLIANCE_ORIENTATION_GAIN);

	// set up the relative transform between the hands
	Trel_pri_to_off = wss[primary_hand]->Tref.inverse() * wss[off_hand]->Tref;

	// set up nullspace stuff
	nullspace_q_refs[Krang::LEFT] = (Krang::Vector7d()   << 0, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_refs[Krang::RIGHT] = (Krang::Vector7d()  << 0,  1.0, 0,  0.5, 0,  0.8, 0).finished();
	nullspace_q_masks[Krang::LEFT] = (Krang::Vector7d()  << 0,    0, 0,    1, 0,    0, 0).finished();
	nullspace_q_masks[Krang::RIGHT] = (Krang::Vector7d() << 0,    0, 0,    1, 0,    0, 0).finished();

	// initialize visualization
	size_t vecsizes[] = {6, 6, 6, 6, 6, 6, 6, 6};
	vis_msg = somatic__visualize_data__alloc(8, vecsizes, 0);
	vis_msg->msg = "teleop/05-workspace";
	somatic_d_channel_open(&daemon_cx, &vis_chan, "teleop-05-workspace-vis", NULL);

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/*
 * Initializes the ach channel for vision stuff
 */
void init_vision()
{
	// create the ach channel
	enum ach_status r = ach_create( "krang_vision", frame_cnt, frame_size, NULL );
	if( ACH_OK != r ) {
		if (ACH_EEXIST == r)
			fprintf( stderr, "Channel already exists.\n" );
		else {
			fprintf( stderr, "Could not create channel: %s\n", ach_result_to_string(r) );
			exit(EXIT_FAILURE);
		}
	}

	// open the channel
	r = ach_open( &vision_chan, "krang_vision", NULL );
	assert(ACH_OK == r);
	r = ach_flush(&vision_chan);
}

/*
 * pulls last set of gripper and table poses from the vision channel
 */
void get_last_vision(Eigen::Vector3d &gripper_pose, Eigen::Vector3d &table_pose)
{
	// receive last
	double rtraj[3][3] = {0};
	int r = ach_get( &vision_chan, &rtraj, sizeof(rtraj), &frame_size, NULL, ACH_O_LAST );

	if ( r == ACH_OK || r == ACH_STALE_FRAMES || r == ACH_MISSED_FRAME ) {
		// Eigen::Vector3d gripper_pose = Eigen::Vector3d(rtraj[0][0], rtraj[0][1], rtraj[0][2]);
		// Eigen::Vector3d table_pose = Eigen::Vector3d(rtraj[1][0], rtraj[1][1], rtraj[1][2]);
		// std::cout << "read gripper pose: " << gripper_pose.transpose() << std::endl;
		// std::cout << "read table pose: " << table_pose.transpose() << std::endl;

		gripper_pose << rtraj[0][0], rtraj[0][1], rtraj[0][2];
		table_pose << rtraj[1][0], rtraj[1][1], rtraj[1][2];
	}
	else {
		std::cout << "read error: " << r << std::endl;
	}
}

void home()
{

	double time_last = aa_tm_timespec2sec(aa_tm_now());

	// open gripper:
	somatic_motor_reset(&daemon_cx, hw->grippers[Krang::LEFT]);
	usleep(1e4);
	somatic_motor_setpos(&daemon_cx, hw->grippers[Krang::LEFT], SCHUNK_GRIPPER_POSITION_OPEN, 1);

	// set a convenient start position for the left arm for pushing the table
	std::cout << "Homing left arm" << std::endl << std::flush;
	Eigen::VectorXd homeConfigsL(7);

	// For pushing square table at original waist height (forgot to record):
	//homeConfigsL << 1.06675184,  -1.12861514, -0.00956137, -2.04222107, 0.12383681,  1.48772061,  -0.06238156;

	// For pushing steel cart at waist height: 2.532  -2.532
	homeConfigsL << 1.00912082, -1.06367242, -0.03891839, -1.88363504, 0.26049611, 1.24310887,  -0.04654793;	

	somatic_motor_setpos(&daemon_cx, hw->arms[Krang::LEFT], homeConfigsL.data(), 7);

	double err = 1e10;
	while (!somatic_sig_received && err > 0.001) {
		// Update robot
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;
		hw->updateSensors(time_delta);

		Eigen::VectorXd q = robot->getConfig(*wss[Krang::LEFT]->arm_ids);
		//std::cout << "q cur: " << q << std::endl;
		err = (homeConfigsL - q).norm();
		//std::cout << "err: " << err << std::endl;
		usleep(1000);
	}

	std::cout << "Should be home now (" << "err=" << err << ")" << std::endl;
}

void run2()
{
	// start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	time_start = aa_tm_timespec2sec(aa_tm_now());

	// some eigen 2D pose vectors for printing vision stuff
	Eigen::Vector3d gripper_pose = Eigen::Vector3d();
	Eigen::Vector3d table_pose = Eigen::Vector3d();

	while(!somatic_sig_received) {
		// ========================================================================================
		// Update state: get passed time and kinematics, sensor input and check for current limit

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// set up debug printing
		debug_print_this_it = (time_now - time_last_display) > (1.0 / DISPLAY_FREQUENCY);
		if(debug_print_this_it) time_last_display = time_now;

		// Update the robot
		hw->updateSensors(time_delta);

		// Check for too high currents
		if(Krang::checkCurrentLimits(hw->arms[Krang::LEFT]->cur, 7)
		   && Krang::checkCurrentLimits(hw->arms[Krang::RIGHT]->cur, 7)) {
			if (sending_commands) {
				sending_commands = false;
				hoh_mode = false;
				somatic_motor_halt(&daemon_cx, hw->arms[Krang::LEFT]);
				somatic_motor_halt(&daemon_cx, hw->arms[Krang::RIGHT]);
			}

			// // TODO: handle this more nicely
			// destroy();
			// exit(EXIT_FAILURE);
		}

		// ========================================================================================
		// Perform workspace for each arm, changing the input for right arm based on synch mode

//		for(int sint = Krang::LEFT; sint-1 < Krang::RIGHT; sint++) {
//			// Get the arm side and the joint angles
//			Krang::Side sde = static_cast<Krang::Side>(sint);
//			Krang::Side sde_other = (sde==Krang::LEFT) ? Krang::RIGHT : Krang::LEFT;
//			Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

		Krang::Side sde = Krang::LEFT;
		Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

		// Create variables
		Eigen::MatrixXd Tref_off_sync;
		Eigen::VectorXd spacenav_input;
		Eigen::VectorXd xdot_spacenav = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd xdot_hoh = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd xdot_ws_goal = Eigen::VectorXd::Zero(6);

		// Nullspace: construct a qdot that the jacobian will bias toward using the nullspace
		nullspace_qdot_refs[sde] = (nullspace_q_refs[sde] - q).cwiseProduct(nullspace_q_masks[sde]);

		// update spacenav, because we use it for grippers regardless of the mode
		//spacenav_input = spnavs[sde]->updateSpaceNav();
		spacenav_input = Eigen::VectorXd::Zero(6);
		spacenav_input << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;


		// get a workspace velocity from the spacenav
		xdot_spacenav = wss[sde]->uiInputVelToXdot(spacenav_input);

		// put together the inputs from spacenav and hand-over-hand
		xdot_ws_goal = xdot_spacenav; // + xdot_hoh;

		// Jacobian: compute the desired jointspace velocity from the inputs and sensors
		Eigen::VectorXd qdot_jacobian;
		wss[sde]->updateFromXdot(xdot_ws_goal, hw->fts[sde]->lastExternal,
								 nullspace_qdot_refs[sde], time_delta, qdot_jacobian);

		// make sure we're not going too fast
		double magn = qdot_jacobian.norm();
		if (magn > 0.5) qdot_jacobian *= (0.5 / magn);
		if (magn < .05) qdot_jacobian *= 0.0;

		// avoid joint limits
		Eigen::VectorXd qdot_avoid(7);
		Krang::computeQdotAvoidLimits(robot, *wss[sde]->arm_ids, q, qdot_avoid);

		// add qdots together to get the overall movement
		Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;

		// pull vision channel info, just for printing
		get_last_vision(gripper_pose, table_pose);

		// apply that to the arm
		Eigen::VectorXd w = wss[sde]->endEffector->getWorldTransform().topRightCorner<3,1>();
		if(sending_commands) {
			std::cout << std::endl;
			std::cout << "<qdot: " << qdot_apply << ">" << std::endl;
			std::cout << "<FT: " << hw->fts[sde]->lastExternal << ">" << std::endl;
			std::cout << "<q:" << q << ">" << std::endl;
			std::cout << "<w:" << w << ">" << std::endl;
			std::cout << "<time:" << time_now-time_start << ">" << std::endl;
			std::cout << "gripper_pose:" << gripper_pose.transpose() << std::endl;
			std::cout << "table_pose:" << table_pose.transpose() << std::endl;
			somatic_motor_setvel(&daemon_cx, hw->arms[sde], qdot_apply.data(), 7);
		}

		// end once we've pushed forward enough
		if (w[0] > 0.6) { // 65
			destroy();
			exit(EXIT_FAILURE);
		}
	}
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	cx_t cx;
	memset(&cx, 0, sizeof(cx));

	// Set the default options
	cx.opt_home_mode = 0;

	// parse cmd args
	argp_parse (&argp, argc, argv, 0, NULL, &cx);

	init();
	init_vision();

	//run();

	if (cx.opt_home_mode)
		home();
	else
		run2();

	destroy();
}
