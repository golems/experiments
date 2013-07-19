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
#include "LibertyClient.h"

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
LibertyClient liberty;

// Workspace control object
WorkspaceControl wrkCtl;

// Krang the monster
KrangControl krang;

// node pointers
kinematics::BodyNode *eeNodeL;
kinematics::BodyNode *eeNodeR;

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
		motor_input_mode = !motor_input_mode;
		if (motor_input_mode) {
			krang.initSomatic();
			krang.updateKrangSkeleton();
			wrkCtl.initializeTransforms();
		}

	}

	if (buttons[id_button_ToggleMotorOutputMode] == 1) {
		motor_output_mode = !motor_output_mode;
		krang.setMotorOutputMode(!(motor_input_mode && !motors_initialized));

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
	mWorld = dl.parseWorld("../../common/scenes/05-World-Krang-Teleop.urdf");
	if (mWorld == NULL)
		mWorld = dl.parseWorld("common/scenes/05-World-Krang-Teleop.urdf"); // for eclipse
	assert((mWorld != NULL) && "Could not find the world");

	eeNodeL = mWorld->getSkeleton("Krang")->getNode("lGripper");
	eeNodeR = mWorld->getSkeleton("Krang")->getNode("rGripper");

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
	int libchans[] = {0,1};
	liberty.initLiberty(&daemon_cx, "liberty", 2, libchans);

	// initialize krang the monster
	krang.initialize(mWorld, &daemon_cx, "Krang");

	// initialize workspace controller
	wrkCtl.initialize(&krang);

	// Manually set the initial arm configuration for the left arm
	VectorXd larm_conf(7), rarm_conf(7);
	larm_conf << 0.0, -M_PI / 3.0, 0.0, -M_PI / 3.0, 0.0, M_PI/6.0, 0.0;
	rarm_conf << 0.0, M_PI / 3.0, 0.0, M_PI / 3.0, 0.0, -M_PI/6.0, 0.0;
	krang.setArmConfig(LEFT_ARM, larm_conf);
	krang.setArmConfig(RIGHT_ARM, rarm_conf);
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
		krang.updateKrangSkeleton();

	VectorXd cfgL = spn1.getConfig(0.1,0.3);
	VectorXd cfgR = spn2.getConfig(0.1,0.3);

	wrkCtl.updateXrefFromXdot(LEFT_ARM, cfgL);

	if (right_track_left_mode)
		wrkCtl.updateXrefFromOther(RIGHT_ARM, LEFT_ARM);
	else
		wrkCtl.updateXrefFromXdot(RIGHT_ARM, cfgR);

	// get xdot from references and force sensor
	VectorXd xdotToRefL = wrkCtl.getXdotFromXref(LEFT_ARM);
	VectorXd xdotToRefR = wrkCtl.getXdotFromXref(RIGHT_ARM);

	// grab FT readings and combine with xdotToRef
	VectorXd xdotL = xdotToRefL;
	VectorXd xdotR = xdotToRefR;
	if (motor_input_mode) {
		double ft_gain = 0.0;
		xdotL += krang.getFtWorldWrench(LEFT_ARM) * ft_gain * dt;
		xdotR += krang.getFtWorldWrench(RIGHT_ARM) * ft_gain * dt;
	}

	// get current arm configurations (for jacobian nullspace)
	VectorXd qL = krang.getArmConfig(LEFT_ARM);
	VectorXd qR = krang.getArmConfig(RIGHT_ARM);

	VectorXd qdotL = wrkCtl.xdotToQdot(LEFT_ARM,  xdotL, 0.01);
	VectorXd qdotR = wrkCtl.xdotToQdot(RIGHT_ARM, xdotR, 0.01);

	krang.setRobotArmVelocities(LEFT_ARM, qdotL, dt);
	krang.setRobotArmVelocities(RIGHT_ARM, qdotR, dt);

	// handle grippers
	VectorXi buttons1 = spn1.getButtons();
	VectorXi buttons2 = spn2.getButtons();
	krang.setRobotiqGripperAction(LEFT_ARM, buttons1);
	krang.setRobotiqGripperAction(RIGHT_ARM, buttons2);
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


