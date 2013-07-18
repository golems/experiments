/**
 * @file 03-workspace.cpp
 * @author Jon Scholz
 * @date July 12, 2013
 * @brief This file demonstrates how to visualize the motion of the arms in grip,
 * under workspace control from a joystick or liberty UI device
 */

#define protected public
#define private public

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include "KrangControl.h"
#include "WorkspaceControl.h"
#include "SpacenavClient.h"
#include "JoystickClient.h"

//old:
//#include "liberty_client.h"
//#include "joystick_client.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;
using namespace simulation;

/* ********************************************************************************************* */

// somatic daemon
somatic_d_t daemon_cx;

// dart world
World* mWorld = NULL;

// UI teleop objects
SpacenavClient spn1;
SpacenavClient spn2;
JoystickClient joystick;

// Workspace control object
WorkspaceControl wrkCtl;

// Krang the monster
KrangControl krang;

// UI Globals
/* Events */
enum BUTTON_EVENTS {
	id_button_ToggleMotorInputMode = 0,
	id_button_ToggleMotorOutputMode,
	id_button_ToggleRightTrackLeftMode,
};

// control globals
enum UI_MODES {
	UI_LIBERTY = 0,
	UI_JOYSTICK,
	UI_MIXED,
};
static bool motor_input_mode = 0;
static bool motor_output_mode = 0;
static bool motors_initialized = 0;
static bool right_track_left_mode = 0;
static int ui_input_mode = UI_JOYSTICK;


/**
 * @function OnButton
 * @brief Handles button events
 */
void handleButtons(VectorXi &buttons) {

	if (buttons[id_button_ToggleMotorInputMode] == 1) {
		motor_input_mode = true;
		krang.setControlMode(!(motor_input_mode && !motors_initialized));
	}

	if (buttons[id_button_ToggleMotorOutputMode] == 1) {

		motor_output_mode = true;
		krang.setControlMode(!(motor_output_mode && !motors_initialized));

		if (!motor_output_mode)
			krang.halt();

	}

	if (buttons[id_button_ToggleRightTrackLeftMode] == 1) {
		right_track_left_mode = true;
		if (right_track_left_mode)
			wrkCtl.updateRelativeTransforms();
	}

}

void init() {
	DartLoader dl;
	mWorld = dl.parseWorld("../../common/scenes/04-World-Liberty.urdf");
	if (mWorld == NULL)
		mWorld = dl.parseWorld("common/scenes/04-World-Liberty.urdf"); // for eclipse
	assert((mWorld != NULL) && "Could not find the world");


	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "03-workspace-teleop";
	somatic_d_init(&daemon_cx, &dopt);

	// Set the interrupt handler
	somatic_sighandler_simple_install();

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Initialize the joystick(s)
	spn1.initialize(&daemon_cx, "spacenav-data");
	spn2.initialize(&daemon_cx, "spacenav2-data");
	joystick.initialize(&daemon_cx, "joystick-data");

	// Initialize the liberty
	//initLiberty();

	// initialize krang the monster
	krang.initialize(mWorld, &daemon_cx, !(motor_input_mode || motor_output_mode));

	// initialize workspace controller
	wrkCtl.initialize(mWorld->getSkeleton("Krang")->getNode("lGripper"), mWorld->getSkeleton("Krang")->getNode("rGripper"));

	// Manually set the initial arm configuration for the left arm
	VectorXd larm_conf(7), rarm_conf(7);
	larm_conf << 0.0, -M_PI / 3.0, 0.0, -M_PI / 3.0, 0.0, M_PI/6.0, 0.0;
	rarm_conf << 0.0, M_PI / 3.0, 0.0, M_PI / 3.0, 0.0, -M_PI/6.0, 0.0;
	krang.setArmConfig(mWorld, LEFT_ARM, larm_conf);
	krang.setArmConfig(mWorld, RIGHT_ARM, rarm_conf);
}

void destroy() {
	krang.halt();
	motors_initialized = 0;

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Clean up the daemon resources
	somatic_d_destroy(&daemon_cx);
}

void step() {
	// Compute timestep
	static double last_movement_time = aa_tm_timespec2sec(aa_tm_now());
	double current_time = aa_tm_timespec2sec(aa_tm_now());
	double dt = current_time - last_movement_time;
	last_movement_time = current_time;


	// update robot state from ach if we're controlling the actual robot
	if (motor_input_mode)
		krang.updateKrangSkeleton(mWorld);

	// handle grippers
	//	VectorXi buttons = getSpacenavButtons(spn_chan);
	//	cout << "buttonsL "<< buttons.transpose() << endl;
	//	handleSpacenavButtons(buttons, rqd_chan);

	VectorXd cfgL = spn1.getConfig() * 0.2;
	VectorXd cfgR = spn2.getConfig() * 0.2;

	wrkCtl.updateXrefFromXdot(LEFT_ARM, cfgL);

	if (right_track_left_mode)
		wrkCtl.updateXrefFromOther(RIGHT_ARM, LEFT_ARM);
	else
		wrkCtl.updateXrefFromXdot(RIGHT_ARM, cfgR);

	VectorXd qL = krang.getArmConfig(mWorld, LEFT_ARM);
	VectorXd qR = krang.getArmConfig(mWorld, RIGHT_ARM);
	VectorXd qdotL = wrkCtl.xdotToQdot(LEFT_ARM, 5.0, 0.01, &qL, NULL);
	VectorXd qdotR = wrkCtl.xdotToQdot(RIGHT_ARM, 5.0, 0.01, &qR, NULL);

	krang.setRobotArmVelocities(mWorld, LEFT_ARM, qdotL, dt);
	krang.setRobotArmVelocities(mWorld, RIGHT_ARM, qdotR, dt);
}

void run() {
	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {
		step();
	}
}

int main() {
	init();
	run();
	destroy();
	return 0;
}


