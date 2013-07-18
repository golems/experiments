/**
 * @file 03-workspace.cpp
 * @author Jon Scholz
 * @date July 12, 2013
 * @brief This file demonstrates how to visualize the motion of the arms in grip,
 * under workspace control from a joystick or liberty UI device
 */

#define protected public
#define private public

#include "simTab.h"
#include "GRIPApp.h"

#include <math/UtilsRotation.h>

#include "util.h"
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

/* ********************************************************************************************* */
// somatic daemon
somatic_d_t daemon_cx;

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
enum DynamicSimulationTabEvents {
	id_button_ResetScene = 8100,
	id_button_ResetLiberty,
	id_button_ResetJoystick,
	id_button_SetInitialTransforms,
	id_checkbox_ToggleMotorInputMode,
	id_checkbox_ToggleMotorOutputMode,
	id_checkbox_ToggleRightTrackLeftMode,
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

// Pointers to frequently used dart data structures
kinematics::BodyNode *eeNodeL;
kinematics::BodyNode *eeNodeR;
dynamics::SkeletonDynamics *goalSkelL;
dynamics::SkeletonDynamics *goalSkelR;


/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {

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

	goalSkelL->setConfig(dartRootDofOrdering, transformToEuler(wrkCtl.getXref(LEFT_ARM), math::XYZ));
	goalSkelR->setConfig(dartRootDofOrdering, transformToEuler(wrkCtl.getXref(RIGHT_ARM), math::XYZ));

	VectorXd qL = krang.getArmConfig(mWorld, LEFT_ARM);
	VectorXd qR = krang.getArmConfig(mWorld, RIGHT_ARM);
	VectorXd qdotL = wrkCtl.xdotToQdot(LEFT_ARM, 5.0, 0.01, &qL, NULL);
	VectorXd qdotR = wrkCtl.xdotToQdot(RIGHT_ARM, 5.0, 0.01, &qR, NULL);

	krang.setRobotArmVelocities(mWorld, LEFT_ARM, qdotL, dt);
	krang.setRobotArmVelocities(mWorld, RIGHT_ARM, qdotR, dt);

	// Visualize the arm motion
	viewer->DrawGLScene();
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size,
		long style) : GRIPTab(parent, id, pos, size, style) {
	// ============================================================================
	// Do basic tab layout
	// Create user interface
	wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("UI Input"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Initialization"));
	wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Control Output"));

	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
	wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

	ss1BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleMotorInputMode, wxT("Read Motor State")), 0, wxALL, 1);

	ss2BoxS->Add(new wxButton(this, id_button_ResetScene, wxT("Reset Scene")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetLiberty, wxT("Set Liberty Initial Transforms")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetJoystick, wxT("Set Joystick Initial Transform")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_SetInitialTransforms, wxT("Set Arm Initial Transforms")), 0, wxALL, 1);

	ss3BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleMotorOutputMode, wxT("Send Motor Commands")), 0, wxALL, 1);
	ss3BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleRightTrackLeftMode, wxT("Track left arm with right")), 0, wxALL, 1);

	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

	SetSizer(sizerFull);

	// ============================================================================
	// load hard-coded scene
	frame->DoLoad("../../common/scenes/04-World-Liberty.urdf");
	if (mWorld == NULL)
		frame->DoLoad("common/scenes/04-World-Liberty.urdf"); // for eclipse

	// ============================================================================
	// set viewer
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(0.0, 0.0, -0.7);
	viewer->UpdateCamera();

	// ============================================================================
	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->Start(0.005 * 1e4);

	// ============================================================================
	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "03-workspace";
	somatic_d_init(&daemon_cx, &dopt);

	// Set the interrupt handler
	somatic_sighandler_simple_install();

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);


	// Initialize the joystick(s)
	spn1.initialize(&daemon_cx, "spacenav-data");
	spn2.initialize(&daemon_cx, "spacenav2-data");

	// Initialize the liberty
	//initLiberty();

	// Grab effector node pointer to compute the task space error and the Jacobian
	eeNodeL = mWorld->getSkeleton("Krang")->getNode("lGripper");
	eeNodeR = mWorld->getSkeleton("Krang")->getNode("rGripper");

	// Grab goal skeleton references
	goalSkelL = mWorld->getSkeleton("g1");
	goalSkelR = mWorld->getSkeleton("g2");

	// initialize krang the monster
	krang.initialize(mWorld, &daemon_cx, !(motor_input_mode || motor_output_mode));

	// initialize workspace controller
	wrkCtl.initialize(eeNodeL, eeNodeR);

	// Manually set the initial arm configuration for the left arm
	VectorXd larm_conf(7), rarm_conf(7);
	larm_conf << 0.0, -M_PI / 3.0, 0.0, -M_PI / 3.0, 0.0, M_PI/6.0, 0.0;
	rarm_conf << 0.0, M_PI / 3.0, 0.0, M_PI / 3.0, 0.0, -M_PI/6.0, 0.0;
	krang.setArmConfig(mWorld, LEFT_ARM, larm_conf);
	krang.setArmConfig(mWorld, RIGHT_ARM, rarm_conf);

}

/* ********************************************************************************************* */
SimTab::~SimTab() {

	// halt the arms TODO: why isn't this being called?
	krang.halt();
	motors_initialized = 0;

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Clean up the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() {}

/**
 * @function OnButton
 * @brief Handles button events
 */
void SimTab::OnButton(wxCommandEvent &evt) {
	int slnum = evt.GetId();
	switch(slnum) {
	// Set Start Arm Configuration
	case id_button_ResetScene: {
		break;
	}

	case id_button_ResetLiberty: {
		//MatrixXd *Tlibs[] = {&T_lib1_init, &T_lib2_init};
		//getLibertyPoses(Tlibs, 2, NULL);
		break;
	}

	case id_button_ResetJoystick: {
		break;
		}

	case id_button_SetInitialTransforms: {
		break;
	}

	case id_checkbox_ToggleMotorInputMode: {
		motor_input_mode = evt.IsChecked();
		krang.setControlMode(!(motor_input_mode && !motors_initialized));

		break;
	}

	case id_checkbox_ToggleMotorOutputMode: {

		motor_output_mode = evt.IsChecked();
		krang.setControlMode(!(motor_output_mode && !motors_initialized));

		if (!motor_output_mode)
			krang.halt();
		break;
	}

	case id_checkbox_ToggleRightTrackLeftMode: {
		right_track_left_mode = evt.IsChecked();
		if (right_track_left_mode)
			wrkCtl.updateRelativeTransforms();
		break;
	}

	default: {}
	}
}

/* ********************************************************************************************* */
void SimTab::OnSlider(wxCommandEvent &evt) {}

/* ********************************************************************************************* */
// Handler for events

BEGIN_EVENT_TABLE(SimTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimTab::OnSlider)
//EVT_CHECKBOX(wxEVT_COMMAND_CHECKBOX_CLICKED, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleMotorInputMode, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleMotorOutputMode, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleRightTrackLeftMode, SimTab::OnButton)
END_EVENT_TABLE()

/* ********************************************************************************************* */
// Class constructor for the tab: Each tab will be a subclass of GRIPTab

IMPLEMENT_DYNAMIC_CLASS(SimTab, GRIPTab)

/* ********************************************************************************************* */
// Necessary interface call to create a GRIP executable

/// Application class
extern wxNotebook* tabView;
class mainApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new SimTab(tabView), wxT("Teleop"));
	}
};

IMPLEMENT_APP(mainApp)


