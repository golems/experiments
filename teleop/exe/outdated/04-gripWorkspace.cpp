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
#include "LibertyClient.h"

#define DISPLAY_VECTOR(VEC) std::cout << std::setw(25) << std::left << #VEC; for(int i = 0; i < VEC.size(); i++) std::cout << std::setw(12) << VEC[i]; std::cout << std::endl;

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
LibertyClient liberty;

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
	id_checkbox_ToggleLibertySpacenavMode,
	id_checkbox_ToggleStationaryCurrentMode,
};

enum sliderNames{
	XDOT_GAIN_SLIDER = 1000,
	FT_GAIN_SLIDER,
	UI_GAIN_SLIDER,
};

// control globals
typedef enum {
	UI_NONE= 0,
	UI_SPACENAV,
	UI_LIBERTY,
	UI_JOYSTICK,
	UI_HOMING,
} ui_mode_t;

static ui_mode_t ui_mode = UI_SPACENAV;
static bool motor_input_mode = 0;
static bool motor_output_mode = 0;
static bool motors_initialized = 0;
static bool right_track_left_mode = 0;
double xdot_gain = 1.0;
double ft_gain = 0.02;
double ui_gain = 1.0;

wxCheckBox* checkbox_tracking;
wxCheckBox* checkbox_motor_output_mode;

// Pointers to frequently used dart data structures
kinematics::BodyNode *eeNodeL;
kinematics::BodyNode *eeNodeR;
dynamics::SkeletonDynamics *goalSkelL;
dynamics::SkeletonDynamics *goalSkelR;

// home configs for the arms
VectorXd homeConfigL(7);
VectorXd homeConfigR(7);

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
		liberty.setInitialPoses();
		wrkCtl.initializeTransforms();
		break;
	}

	case id_button_ResetJoystick: {
		wrkCtl.initializeTransforms();
		break;
		}

	case id_button_SetInitialTransforms: {
		break;
	}

	case id_checkbox_ToggleMotorInputMode: {
		motor_input_mode = evt.IsChecked();

		if (motor_input_mode) {
			krang.initSomatic();
			krang.updateKrangSkeleton();
			wrkCtl.initializeTransforms();
		}
		break;
	}

	case id_checkbox_ToggleMotorOutputMode: {
		motor_output_mode = evt.IsChecked();

		if (motor_output_mode) {
			checkbox_tracking->SetValue(false);
			right_track_left_mode = false;
			wrkCtl.initializeTransforms();
		}

		krang.setMotorOutputMode(motor_output_mode);

		break;
	}

	case id_checkbox_ToggleRightTrackLeftMode: {
		right_track_left_mode = evt.IsChecked();
		if (right_track_left_mode) {
			Matrix4d curTL = eeNodeL->getWorldTransform();
			Matrix4d curTR = eeNodeR->getWorldTransform();
			wrkCtl.setXcur(LEFT_ARM, curTL);
			wrkCtl.setXcur(RIGHT_ARM, curTR);
			wrkCtl.updateRelativeTransforms();
		}
		else
			wrkCtl.initializeTransforms();
		break;
	}

	case id_checkbox_ToggleLibertySpacenavMode: {
		if (evt.IsChecked()) {
			ui_mode = UI_LIBERTY;
			liberty.setInitialPoses();
			wrkCtl.initializeTransforms();
		}
		else {
			ui_mode = UI_SPACENAV;
			wrkCtl.initializeTransforms();
		}
		break;
	}

	case id_checkbox_ToggleStationaryCurrentMode: {
//		krang._current_mode = evt.IsChecked();
//		if (krang._current_mode)
//			ui_mode = UI_NONE;
		if (evt.IsChecked())
			ui_mode = UI_HOMING;
		else {
			ui_mode = UI_SPACENAV;
			wrkCtl.initializeTransforms();
		}

		break;
	}
	default: {}
	}
}

void handleSpacenavButtons(const VectorXi &buttons, ach_channel_t &grip_chan) {
	robotiqd_achcommand_t rqd_msg;
	rqd_msg.mode = GRASP_BASIC;
	rqd_msg.grasping_speed = 0xff;
	rqd_msg.grasping_force = 0xff;

	if (buttons[0]) rqd_msg.grasping_pos = 0x00;
	if (buttons[1]) rqd_msg.grasping_pos = 0xff;

	if (buttons[0] || buttons[1])
		ach_put(&grip_chan, &rqd_msg, sizeof(rqd_msg));
}

/* ********************************************************************************************* */
void SimTab::OnSlider(wxCommandEvent &evt) {
	int slnum = evt.GetId();
	double pos = *(double*)evt.GetClientData();
	switch(slnum) {
	      //-- Change joint value
	    case XDOT_GAIN_SLIDER: {
	    	xdot_gain = pos;
	    	cout << "set xdot gain: " << xdot_gain << endl;
	    	break;
	    }
	    case FT_GAIN_SLIDER: {
	    	ft_gain = pos;
	    	cout << "set ft gain: " << ft_gain << endl;
	    	break;
	    }
	    case UI_GAIN_SLIDER: {
	    	ui_gain = pos;
	    	cout << "set ui gain: " << ui_gain << endl;
	    	break;
	    }
	}
}

/* ********************************************************************************************* */
// Handler for events

BEGIN_EVENT_TABLE(SimTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimTab::OnSlider)
//EVT_CHECKBOX(wxEVT_COMMAND_CHECKBOX_CLICKED, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleMotorInputMode, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleMotorOutputMode, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleRightTrackLeftMode, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleLibertySpacenavMode, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleStationaryCurrentMode, SimTab::OnButton)
END_EVENT_TABLE()

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
	if (motor_input_mode) {
		if (!krang.updateKrangSkeleton()) {
			motor_output_mode = false;
			krang.setMotorOutputMode(false);
			checkbox_motor_output_mode->SetValue(false);
		}
	}

	switch (ui_mode) {
	case UI_SPACENAV: {
		VectorXd cfgL = spn1.getConfig(0.02*ui_gain,0.06*ui_gain);
		VectorXd cfgR = spn2.getConfig(0.02*ui_gain,0.06*ui_gain);
		//goalSkelL->setConfig(dartRootDofOrdering, cfgL); // uncomment to visualize spacenav directly

		wrkCtl.updateXrefFromXdot(LEFT_ARM, cfgL);

		if (right_track_left_mode)
			wrkCtl.updateXrefFromOther(RIGHT_ARM, LEFT_ARM);
		else
			wrkCtl.updateXrefFromXdot(RIGHT_ARM, cfgR);
		break;
	}
	case UI_LIBERTY: {
		// read liberty poses
		std::vector<Eigen::Matrix4d> liberty_poses = liberty.getRelPoses(true);
		wrkCtl.setXrefFromOffset(LEFT_ARM, liberty_poses[0]);
		if (right_track_left_mode)
			wrkCtl.updateXrefFromOther(RIGHT_ARM, LEFT_ARM);
		else
			wrkCtl.setXrefFromOffset(RIGHT_ARM, liberty_poses[1]);
		break;
	}
	case UI_HOMING: {
		Eigen::VectorXd qdotL = homeConfigL - krang.getArmConfig(LEFT_ARM);
		Eigen::VectorXd qdotR = homeConfigR - krang.getArmConfig(RIGHT_ARM);
		if (qdotL.norm() > .5) qdotL = qdotL.normalized() * .5;
		if (qdotR.norm() > .5) qdotR = qdotR.normalized() * .5;
		krang.setRobotArmVelocities(LEFT_ARM, qdotL, dt);
		krang.setRobotArmVelocities(RIGHT_ARM, qdotR, dt);
		viewer->DrawGLScene();
		return;
	}
	case UI_NONE:
		break;
	}

	goalSkelL->setConfig(dartRootDofOrdering, transformToEuler(wrkCtl.getXref(LEFT_ARM), math::XYZ));
	goalSkelR->setConfig(dartRootDofOrdering, transformToEuler(wrkCtl.getXref(RIGHT_ARM), math::XYZ));

	// get xdot from references and force sensor
	VectorXd xdotToRefL = wrkCtl.getXdotFromXref(LEFT_ARM);
	VectorXd xdotToRefR = wrkCtl.getXdotFromXref(RIGHT_ARM);

	// grab FT readings and combine with xdotToRef
	VectorXd xdotL = xdotToRefL * xdot_gain;
	VectorXd xdotR = xdotToRefR * xdot_gain;
	if (motor_input_mode) {
		Eigen::VectorXd xdotFtL = krang.getFtWorldWrench(LEFT_ARM) * ft_gain * dt;
		Eigen::VectorXd xdotFtR = krang.getFtWorldWrench(RIGHT_ARM) * ft_gain * dt;
		//DISPLAY_VECTOR(xdotFtL);
		//DISPLAY_VECTOR(xdotFtR);
		xdotL += xdotFtL;
		xdotR += xdotFtR;
	}

	// get current arm configurations (for jacobian nullspace)
	VectorXd qL = krang.getArmConfig(LEFT_ARM);
	VectorXd qR = krang.getArmConfig(RIGHT_ARM);

	VectorXd qdotL = wrkCtl.xdotToQdot(LEFT_ARM,  xdotL, -0.02);
	VectorXd qdotR = wrkCtl.xdotToQdot(RIGHT_ARM, xdotR, -0.02);

	krang.setRobotArmVelocities(LEFT_ARM, qdotL, dt);
	krang.setRobotArmVelocities(RIGHT_ARM, qdotR, dt);

	// handle grippers
	VectorXi buttons1 = spn1.getButtons();
	VectorXi buttons2 = spn2.getButtons();
	krang.setRobotiqGripperAction(LEFT_ARM, buttons1);
	krang.setRobotiqGripperAction(RIGHT_ARM, buttons2);

	// draw in joint angles in color so we can avoid joint limits
	for(int i = 0; i < krang.getKrang()->getNumNodes(); i++) {
		kinematics::BodyNode* node = krang.getKrang()->getNode(i);
		if (node->getNumLocalDofs() == 0) continue;
		kinematics::Dof* d = node->getDof(0);
		kinematics::Shape* sh = node->getShape();
		double joint_min = d->getMin();
		double joint_max = d->getMax();
		double mid = (joint_min + joint_max) / 2;
		double value = fabs(2 * (mid - d->getValue()) / (joint_max- joint_min));
		sh->setColor(Eigen::Vector3d(255 * value, 255 * (1 - value), 0.0));
	}

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
	ss1BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleLibertySpacenavMode, wxT("Use Liberty not spacenav")), 0, wxALL, 1);
	ss1BoxS->Add(new GRIPSlider("UI Gain",0,10,500,ui_gain,100,500,this,UI_GAIN_SLIDER), 0, wxALL, 1);

	ss2BoxS->Add(new wxButton(this, id_button_ResetScene, wxT("Set Arms to Start Config")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetLiberty, wxT("Set Liberty Initial Transforms")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetJoystick, wxT("Reset Reference Transforms")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_SetInitialTransforms, wxT("Empty Button")), 0, wxALL, 1);

	checkbox_motor_output_mode = new wxCheckBox(this, id_checkbox_ToggleMotorOutputMode, wxT("Send Motor Commands"));
	ss3BoxS->Add(checkbox_motor_output_mode, 0, wxALL, 1);
	checkbox_tracking = new wxCheckBox(this, id_checkbox_ToggleRightTrackLeftMode, wxT("Track left arm with right"));
	ss3BoxS->Add(checkbox_tracking, 0, wxALL, 1);
	ss3BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleStationaryCurrentMode, wxT("Homing Mode")), 0, wxALL, 1); // Stationary Current Mode
	ss3BoxS->Add(new GRIPSlider("Xdot Gain",0,10,500,xdot_gain,100,500,this,XDOT_GAIN_SLIDER), 0, wxALL, 1);
	ss3BoxS->Add(new GRIPSlider("FT Gain",0,0.2,500,ft_gain,100,500,this,FT_GAIN_SLIDER), 0, wxALL, 1);

	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

	SetSizer(sizerFull);

	// ============================================================================
	// load hard-coded scene
	frame->DoLoad("../../common/scenes/05-World-Krang-Teleop.urdf");
	if (mWorld == NULL)
		frame->DoLoad("common/scenes/05-World-Krang-Teleop.urdf"); // for eclipse

	// ============================================================================
	// set viewer
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(-0.3, 0.0, -0.8);
		Matrix3d rotM; rotM << -0.845948,  0.0038246,   0.533252,0.000573691,-0.999967,0.00808209, 0.533265, 0.00714295, 0.845918;
	viewer->camRotT = rotM;
	viewer->UpdateCamera();

	// ============================================================================
	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->Start(20);

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
	int libchans[] = {0,1};
	liberty.initLiberty(&daemon_cx, "liberty", 2, libchans);

	// Grab effector node pointer to compute the task space error and the Jacobian
	eeNodeL = mWorld->getSkeleton("Krang")->getNode("lGripper");
	eeNodeR = mWorld->getSkeleton("Krang")->getNode("rGripper");

	// Grab goal skeleton references
	goalSkelL = mWorld->getSkeleton("g1");
	goalSkelR = mWorld->getSkeleton("g2");

	// initialize krang the monster
	krang.initialize(mWorld, &daemon_cx, "Krang");

	// Manually set the initial arm configuration for the left arm
//	VectorXd larm_conf(7), rarm_conf(7);
//	larm_conf << 0.0, -M_PI / 3.0, 0.0, -M_PI / 3.0, 0.0, M_PI/6.0, 0.0;
//	rarm_conf << 0.0, M_PI / 3.0, 0.0, M_PI / 3.0, 0.0, -M_PI/6.0, 0.0;
//	krang.setArmConfig(LEFT_ARM, larm_conf);
//	krang.setArmConfig(RIGHT_ARM, rarm_conf);
	homeConfigL <<  0.2, -0.589,  0.000, -1.339,  0.000, -0.959, -1.000;
	homeConfigR << -1.102,  0.589,  0.000,  1.339,  0.141,  0.959, -1.000;
	krang.setArmConfig(LEFT_ARM, homeConfigL);
	krang.setArmConfig(RIGHT_ARM, homeConfigR);

	// initialize workspace controller
	wrkCtl.initialize(&krang);
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


