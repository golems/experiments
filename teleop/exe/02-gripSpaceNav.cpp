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
 * @file 02-gripSpaceNav.cpp
 * @author Can Erdogan, Saul Reynolds-Haertle, Jon Scholtz
 * @date July 24, 2013
 * @brief This executable shows workspace control with a spacenav in the dart/grip environment.
 * NOTE: Although the spacenav input is interpreted as a workspace velocity, we first create a 
 * workspace reference from it and then compute xdot back. The motivation is under application such
 * as liberty and compliance plays with workspace reference configurations.
 */

// grip stuff
#include <GRIPApp.h>
#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>
#include <Tabs/AllTabs.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>

// dart stuff
#include <Tools/Constants.h>
#include <simulation/World.h>
#include <collision/fcl_mesh/CollisionShapes.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>

// wxwidgets stuff
#include <wx/wx.h>
#include <wx/statbox.h>
#include <wx/tglbtn.h>

// kore
#include <kore/workspace.hpp>
#include <kore/sensors.hpp>
#include <kore/safety.hpp>
#include <kore/display.hpp>

// general stuff stuff
#include <Eigen/Dense>







/* ********************************************************************************************* */
/// Declare a timer class so that we're updating state at a reasonable frequency
class GripSpaceNavUpdateTimer : public wxTimer {
public:
	void Notify ();
};

/* ********************************************************************************************* */
/// Declare the class for our tab
class GripSpaceNavTab : public GRIPTab {
public:
	GripSpaceNavUpdateTimer* timer;

	GripSpaceNavTab(){};									///< Default constructor
	GripSpaceNavTab(wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
	            const wxSize& size = wxDefaultSize, long style = wxTAB_TRAVERSAL);
	virtual ~GripSpaceNavTab();				///< Destructor

	/// handles changing the synch mode on and off
	void OnCheckbox_synch_mode(wxCommandEvent& evt);

	/// handles turning hand-over-hand mode on and off
	void OnCheckbox_hoh_mode(wxCommandEvent& evt);

	/// handles changing which hand is being controlled
	void OnCheckbox_primary_hand_left(wxCommandEvent& evt);
	void OnCheckbox_primary_hand_right(wxCommandEvent& evt);

	/// handles changing which direction hand-over-hand mode should move
	void OnCheckbox_hoh_direction_left(wxCommandEvent& evt);
	void OnCheckbox_hoh_direction_right(wxCommandEvent& evt);

	/// updates the primary and off hand refs and the transform between the primary and off hands
	void update_hand_relative_transform();
	
public:
	DECLARE_DYNAMIC_CLASS(SimTab)
	DECLARE_EVENT_TABLE()
};








/* ********************************************************************************************* */
/// Constants

// initializers for workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.75;
const double SPACENAV_TRANSLATION_GAIN = 0.25; 
const double COMPLIANCE_TRANSLATION_GAIN = 1.0 / 750.0;
const double COMPLIANCE_ORIENTATION_GAIN = .125 / 750.0;
const double HAND_OVER_HAND_SPEED = 0.05; // 3 cm per second when going hand-over-hand

// various rate limits
const double LOOP_FREQUENCY = 10.0;
const double DISPLAY_FREQUENCY = 3.0;

// debug information
bool debug_print_this_it;       ///< whether we print

// ui event ids
enum GripSpaceNavTabEvents {
	event_checkbox_synch_mode = 23857,
	event_checkbox_hoh_mode,
	event_checkbox_primary_hand_left,
	event_checkbox_primary_hand_right,
	event_checkbox_hoh_direction_left,
	event_checkbox_hoh_direction_right,
};

/* ********************************************************************************************* */
// Variables

// daemon things
somatic_d_t daemon_cx; ///< process context
std::map<Krang::Side, Krang::SpaceNav*> spnavs; ///< points to spacenavs
double time_last_display;
double time_last;

// pointers to important DART objects
dynamics::SkeletonDynamics* robot;

// workspace stuff
std::map<Krang::Side, Krang::WorkspaceControl*> wss; ///< does workspace control for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_q_refs; ///< nullspace configurations for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_q_masks; ///< nullspace configurations for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_qdot_refs; ///< nullspace configurations for the arms
Eigen::MatrixXd Trel_pri_to_off; ///< translation from the primary hand to the off hand

// hand-over-hand stuff
std::map<Krang::Side, Eigen::Vector3d> hoh_initpos;

// UI elements
wxCheckBox* checkbox_synch_mode;
wxCheckBox* checkbox_hoh_mode;
wxCheckBox* checkbox_primary_hand_left;
wxCheckBox* checkbox_primary_hand_right;
wxCheckBox* checkbox_hoh_moving_left;
wxCheckBox* checkbox_hoh_moving_right;







/* ********************************************************************************************* */
/// Constructor for the tab
GripSpaceNavTab::GripSpaceNavTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size,
               long style) : GRIPTab(parent, id, pos, size, style) {

	// ============================================================================
	// set up and lay out the UI

	// create layout objects
	wxBoxSizer* tab_sizer = new wxBoxSizer(wxHORIZONTAL);
	wxStaticBoxSizer* checkboxes_box_sizer = new wxStaticBoxSizer(new wxStaticBox(this, -1, wxT("Mode switching")), wxVERTICAL);
	wxStaticBoxSizer* primary_hand_box_sizer = new wxStaticBoxSizer(new wxStaticBox(this, -1, wxT("Primary hand")), wxHORIZONTAL);
	wxStaticBoxSizer* hoh_dir_box_sizer = new wxStaticBoxSizer(new wxStaticBox(this, -1, wxT("Hand-over-hand dir")), wxHORIZONTAL);
	tab_sizer->Add(checkboxes_box_sizer);

	// add a load of checkboxes
	checkbox_synch_mode = new wxCheckBox(this, event_checkbox_synch_mode, wxT("Synch mode"));
	checkbox_hoh_mode = new wxCheckBox(this, event_checkbox_hoh_mode, wxT("Hand-over-hand mode"));
	checkbox_primary_hand_left = new wxCheckBox(this, event_checkbox_primary_hand_left, wxT("Left"));
	checkbox_primary_hand_right = new wxCheckBox(this, event_checkbox_primary_hand_right, wxT("Right"));
	checkbox_primary_hand_left->SetValue(true);
	checkbox_hoh_moving_left = new wxCheckBox(this, event_checkbox_hoh_direction_left, wxT("Left"));
	checkbox_hoh_moving_right = new wxCheckBox(this, event_checkbox_hoh_direction_right, wxT("Right"));
	checkbox_hoh_moving_left->SetValue(true);

	// and lay them out
	checkboxes_box_sizer->Add(checkbox_synch_mode, 0, wxALL, 1);
	checkboxes_box_sizer->Add(checkbox_hoh_mode, 0, wxALL, 1);
	primary_hand_box_sizer->Add(checkbox_primary_hand_left, 0, wxALL, 1);
	primary_hand_box_sizer->Add(checkbox_primary_hand_right, 0, wxALL, 1);
	checkboxes_box_sizer->Add(primary_hand_box_sizer);
	hoh_dir_box_sizer->Add(checkbox_hoh_moving_left, 0, wxALL, 1);
	hoh_dir_box_sizer->Add(checkbox_hoh_moving_right, 0, wxALL, 1);
	checkboxes_box_sizer->Add(hoh_dir_box_sizer);

	// and finalize the layout
	SetSizer(tab_sizer);

	// set the initial camera angle
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(-0.3, 0.0, -0.8);
	Matrix3d rotM; 
	rotM << -0.8459, 0.0038246, 0.5332,0.000573691,-0.9999,0.008082, 0.533265, 0.00714295, 0.845918;
	viewer->camRotT = rotM;
	viewer->UpdateCamera();

	// ============================================================================
	// start running

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	mWorld = dl.parseWorld("../../common/scenes/05-World-Teleop.urdf");
	assert((mWorld != NULL) && "Could not find the world");
	robot = mWorld->getSkeleton("Krang");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "02-gripSpaceNav";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the spacenav
	spnavs[Krang::LEFT] = new Krang::SpaceNav(&daemon_cx, "spacenav-data-l", .5);
	spnavs[Krang::RIGHT] = new Krang::SpaceNav(&daemon_cx, "spacenav-data-r", .5);

	// Also, set the imu and waist angles
	std::vector <int> imuWaist_ids;
	imuWaist_ids.push_back(5);
	imuWaist_ids.push_back(8);
	Vector2d imuWaist (3.45, 2.81);
	robot->setConfig(imuWaist_ids, imuWaist);

	// Set up the workspace controllers
	wss[Krang::LEFT] = new Krang::WorkspaceControl(robot, Krang::LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
	                                               SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN,
	                                               COMPLIANCE_TRANSLATION_GAIN, COMPLIANCE_ORIENTATION_GAIN);
	wss[Krang::RIGHT] = new Krang::WorkspaceControl(robot, Krang::RIGHT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
	                                                SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN,
	                                                COMPLIANCE_TRANSLATION_GAIN, COMPLIANCE_ORIENTATION_GAIN);

	// Manually set the initial arm configuration for the arms
	Eigen::VectorXd homeConfigsL(7);
	homeConfigsL <<  0.97, -0.589,  0.000, -1.339,  0.000, -0.959, -1.000;
	Eigen::VectorXd homeConfigsR(7);
	homeConfigsR << -1.102,  0.589,  0.000,  1.339,  0.141,  0.959, -1.000;
	robot->setConfig(*wss[Krang::LEFT]->arm_ids, homeConfigsL);
	robot->setConfig(*wss[Krang::RIGHT]->arm_ids, homeConfigsR);
	wss[Krang::LEFT]->resetReferenceTransform();
	wss[Krang::RIGHT]->resetReferenceTransform();

	// and make the goal skeletons go somewhere else
	Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
	mWorld->getSkeleton("g1")->setConfig(Krang::dart_root_dof_ids, zero);
	mWorld->getSkeleton("g2")->setConfig(Krang::dart_root_dof_ids, zero);
	mWorld->getSkeleton("g3")->setConfig(Krang::dart_root_dof_ids, zero);
	mWorld->getSkeleton("g4")->setConfig(Krang::dart_root_dof_ids, zero);

	// set up nullspace stuff
	nullspace_q_refs[Krang::LEFT] = (Krang::Vector7d()   << 0, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_refs[Krang::RIGHT] = (Krang::Vector7d()  << 0,  1.0, 0,  0.5, 0,  0.8, 0).finished();
	nullspace_q_masks[Krang::LEFT] = (Krang::Vector7d()  << 0,    0, 0,    1, 0,    0, 0).finished();
	nullspace_q_masks[Krang::RIGHT] = (Krang::Vector7d() << 0,    0, 0,    1, 0,    0, 0).finished();

	// set up the relative transform between the hands
	update_hand_relative_transform();

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// and start looping
	time_last = aa_tm_timespec2sec(aa_tm_now());
	time_last_display = aa_tm_timespec2sec(aa_tm_now());
	timer = new GripSpaceNavUpdateTimer();
	timer->Start(1);
}

/* ********************************************************************************************* */
GripSpaceNavTab::~GripSpaceNavTab() {

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// close workspace stuff
	delete wss[Krang::LEFT];
	delete wss[Krang::RIGHT];

	// close spacenav stuff
	delete spnavs[Krang::LEFT];
	delete spnavs[Krang::RIGHT];

	// Clean up the daemon resources
	somatic_d_destroy(&daemon_cx);
}




/* ********************************************************************************************* */
/// updates the primary and off hand refs and the transform between the primary and off hands
void GripSpaceNavTab::update_hand_relative_transform() {
	Krang::Side primary_hand = checkbox_primary_hand_left->IsChecked() ? Krang::LEFT : Krang::RIGHT;
	Krang::Side off_hand = checkbox_primary_hand_left->IsChecked() ? Krang::RIGHT : Krang::LEFT;
	Trel_pri_to_off = wss[primary_hand]->Tref.inverse() * wss[off_hand]->Tref;
}

/* ********************************************************************************************* */
/// handles changing the synch mode on and off
void GripSpaceNavTab::OnCheckbox_synch_mode(wxCommandEvent& evt) {
	update_hand_relative_transform();
}

/* ********************************************************************************************* */
/// handles turning hand-over-hand mode on and off
void GripSpaceNavTab::OnCheckbox_hoh_mode(wxCommandEvent& evt) {
	hoh_initpos[Krang::RIGHT] = wss[Krang::RIGHT]->endEffector->getWorldTransform().topRightCorner<3,1>();
	hoh_initpos[Krang::LEFT] = wss[Krang::LEFT]->endEffector->getWorldTransform().topRightCorner<3,1>();
}

/* ********************************************************************************************* */
/// handles changing which hand is being controlled
void GripSpaceNavTab::OnCheckbox_primary_hand_left(wxCommandEvent& evt) {
	checkbox_primary_hand_right->SetValue(!checkbox_primary_hand_right->IsChecked());
	update_hand_relative_transform();
}
void GripSpaceNavTab::OnCheckbox_primary_hand_right(wxCommandEvent& evt) {
	checkbox_primary_hand_left->SetValue(!checkbox_primary_hand_left->IsChecked());
	update_hand_relative_transform();
}

/* ********************************************************************************************* */
/// handles changing which direction hand-over-hand mode should move
void GripSpaceNavTab::OnCheckbox_hoh_direction_left(wxCommandEvent& evt) {
	checkbox_hoh_moving_right->SetValue(!checkbox_hoh_moving_right->IsChecked());
}
void GripSpaceNavTab::OnCheckbox_hoh_direction_right(wxCommandEvent& evt) {
	checkbox_hoh_moving_left->SetValue(!checkbox_hoh_moving_left->IsChecked());
}




/* ********************************************************************************************* */
/// Gets the workspace velocity input from the spacenav, updates a workspace reference position
/// and moves the left arm end-effector to that position
void GripSpaceNavUpdateTimer::Notify() {

	// ============================================================================
	// Handle some timing and debug stuff

	// Update times
	double time_now = aa_tm_timespec2sec(aa_tm_now());
	double time_delta = time_now - time_last;
	time_last = time_now;

	// set up debug prints
	debug_print_this_it = (time_now - time_last_display) > (1.0 / DISPLAY_FREQUENCY);
	if (debug_print_this_it) time_last_display = time_now;

	// cache this since we use it all over the place 
	Krang::Side primary_hand = checkbox_primary_hand_left->IsChecked() ? Krang::LEFT : Krang::RIGHT;
	Krang::Side off_hand = checkbox_primary_hand_left->IsChecked() ? Krang::RIGHT : Krang::LEFT;

	// ========================================================================================
	// Perform workspace for each arm, changing the input for right arm based on synch mode
	for(int sint = Krang::LEFT; sint-1 < Krang::RIGHT; sint++) {
		// Get the arm side and the joint angles
		Krang::Side sde = static_cast<Krang::Side>(sint);
		Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

		if (debug_print_this_it) {
			std::cout << "Arm: " << ((sde == Krang::LEFT) ? "LEFT" : "RIGHT") << std::endl;
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
		spacenav_input = spnavs[sde]->updateSpaceNav();

		// depending on the synch mode, get a workspace velocity either from the spacenav or
		// from the other arm
		if(checkbox_synch_mode->IsChecked() && sde == off_hand) {
			Tref_off_sync = wss[primary_hand]->Tref * Trel_pri_to_off;
		} else if (!checkbox_hoh_mode->IsChecked()) {
			xdot_spacenav = wss[sde]->uiInputVelToXdot(spacenav_input);
		} else if (checkbox_hoh_mode->IsChecked() && sde == primary_hand) {
			Eigen::Vector3d dx_hoh = hoh_initpos[Krang::RIGHT] - hoh_initpos[Krang::LEFT];
			// since that translation is from left to right, if we want to move right to left we
			// have to turn it around
			if (checkbox_hoh_moving_left->IsChecked())
				dx_hoh *= -1;
			xdot_hoh = Eigen::VectorXd::Zero(6);
			xdot_hoh.topLeftCorner<3,1>() = dx_hoh.normalized() * HAND_OVER_HAND_SPEED;
		}

		// put together the inputs from spacenav and hand-over-hand
		xdot_ws_goal = xdot_spacenav + xdot_hoh;

		// Jacobian: compute the desired jointspace velocity from the inputs and sensors
		Eigen::VectorXd qdot_jacobian;
		Eigen::VectorXd ft_last_external = Eigen::VectorXd::Zero(6);
		if(checkbox_synch_mode->IsChecked() && (sde == off_hand)) {
			wss[sde]->updateFromUIPos(Tref_off_sync, ft_last_external,
			                          nullspace_qdot_refs[sde], qdot_jacobian);
		}
		else {
			wss[sde]->updateFromXdot(xdot_ws_goal, ft_last_external,
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
		q += qdot_apply * time_delta;
		robot->setConfig(*wss[sde]->arm_ids, q);
	}

	// Visualize the reference positions
	VectorXd refConfigL = Krang::transformToEuler(wss[Krang::LEFT]->Tref, math::XYZ);
	mWorld->getSkeleton("g1")->setConfig(Krang::dart_root_dof_ids, refConfigL);
	VectorXd refConfigR = Krang::transformToEuler(wss[Krang::RIGHT]->Tref, math::XYZ);
	mWorld->getSkeleton("g2")->setConfig(Krang::dart_root_dof_ids, refConfigR);

	// Visualize the scene
	viewer->DrawGLScene();

	// and start the timer for the next iteration
	double time_loop_end = aa_tm_timespec2sec(aa_tm_now());
	double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loop_end - time_now);
	int time_sleep_msecs = std::max(0, (int)(time_sleep * 1e3));
	Start(time_sleep_msecs);
}

/* ********************************************************************************************* */
// Handler for events

BEGIN_EVENT_TABLE(GripSpaceNavTab, wxPanel)
EVT_CHECKBOX(event_checkbox_synch_mode, GripSpaceNavTab::OnCheckbox_synch_mode)
EVT_CHECKBOX(event_checkbox_hoh_mode, GripSpaceNavTab::OnCheckbox_hoh_mode)
EVT_CHECKBOX(event_checkbox_primary_hand_left, GripSpaceNavTab::OnCheckbox_primary_hand_left)
EVT_CHECKBOX(event_checkbox_primary_hand_right, GripSpaceNavTab::OnCheckbox_primary_hand_right)
EVT_CHECKBOX(event_checkbox_hoh_direction_left, GripSpaceNavTab::OnCheckbox_hoh_direction_left)
EVT_CHECKBOX(event_checkbox_hoh_direction_right, GripSpaceNavTab::OnCheckbox_hoh_direction_right)
END_EVENT_TABLE()

/* ********************************************************************************************* */
// Class constructor for the tab: Each tab will be a subclass of GRIPTab

IMPLEMENT_DYNAMIC_CLASS(GripSpaceNavTab, GRIPTab)

/* ********************************************************************************************* */
// Necessary interface call to create a GRIP executable

/// Application class
extern wxNotebook* tabView;
class mainApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new GripSpaceNavTab(tabView), wxT("Teleop"));
	}
};

IMPLEMENT_APP(mainApp)


