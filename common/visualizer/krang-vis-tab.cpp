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
 * @file krang-vis-tab.cpp
 * @author Saul Reynolds-Haertle
 * @date Aug 05, 2013
 * @brief This tab visualizes the current state of krang, including (if that data is available) the
 * internal state of the examples which publish that information for visualization.
 */

/* ############################################################################################# */
/* ############################################################################################# */
/// Includes

#include "krang-vis-tab.hpp"
#include <math/UtilsRotation.h>
#include <simulation/World.h>
#include <yui/GLFuncs.h>

#include <kore.hpp>
#include <kore/display.hpp>

#include <somatic/msg.h>

/* ############################################################################################# */
/* ############################################################################################# */
// type definitions

/* ############################################################################################# */
/* ############################################################################################# */
// constants

const double DISPLAY_FREQUENCY = 30.0; ///< frequency with which we update our display

/// a list of the ids we'll attach to various events
enum KrangVisTabEvents {
	event_checkbox_vis_krang_body = wxID_HIGHEST + 293847,
	event_checkbox_vis_05_workspace_teleop,
};

/* ############################################################################################# */
/* ############################################################################################# */
// variables

// communication
somatic_d_t daemon_cx; ///< daemon context
ach_channel_t vis_chan; ///< ach channel we use for getting data to be visualized
Krang::Hardware* hw; ///< hardware object that we use to talk to krang's general body state
Somatic__VisualizeData* vis_msg_cached; ///< hang on to this because sometimes we don't get a message

// timing information
double time_last;

// dart
dynamics::SkeletonDynamics* robot; ///< pointer to the skeleton dart uses for krang
std::map<Krang::Side, dynamics::SkeletonDynamics*> goal_skels;
std::map<Krang::Side, kinematics::BodyNode*> end_effs;

// user interface
wxCheckBox* checkbox_vis_krang_body; ///< pointer to the checkbox for visaulizing krang's body
wxCheckBox* checkbox_vis_05_workspace_teleop; ///< pointer to the checkbox for visualizng workspace teleop

/* ############################################################################################# */
/* ############################################################################################# */
// function declarations

/* ############################################################################################# */
/* ############################################################################################# */
// function definitions

/* ********************************************************************************************* */
/// Constructor. Set up the UI and lay out UI objects, initialize the daemon, and start looping.
KrangVisTab::KrangVisTab(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
	: GRIPTab(parent, id, pos, size, style) {
	// ==============================
	// set up and lay out the UI

	// create layout objects
	wxSizer* tab_sizer = new wxBoxSizer(wxHORIZONTAL);
	wxStaticBox* checkboxes_box = new wxStaticBox(this, -1, wxT("Visualize:"));
	wxStaticBoxSizer* checkboxes_box_sizer = new wxStaticBoxSizer(checkboxes_box, wxVERTICAL);
	tab_sizer->Add(checkboxes_box_sizer);
	
	// make some checkboxes
	checkbox_vis_krang_body = new wxCheckBox(this, event_checkbox_vis_krang_body, wxT("Krang Body"));
	checkbox_vis_05_workspace_teleop = new wxCheckBox(this, event_checkbox_vis_05_workspace_teleop, wxT("Workspace Teleop"));
	
	// add our checkboxes to the checkbox box
	checkboxes_box_sizer->Add(checkbox_vis_krang_body, 0, wxALL, 1);
	checkboxes_box_sizer->Add(checkbox_vis_05_workspace_teleop, 0, wxALL, 1);
	
	// finalize the layout
	SetSizer(tab_sizer);

	// choose the initial camera angle
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(-0.3, 0.0, -0.8);
	Matrix3d rotM; rotM << -0.845948,  0.0038246,   0.533252,0.000573691,-0.999967,0.00808209, 0.533265, 0.00714295, 0.845918;
	viewer->camRotT = rotM;
	viewer->UpdateCamera();

	// ==============================
	// start running

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon
	// changes our current directory to somewhere in /var/run.
	frame->DoLoad("../../scenes/05-World-Teleop.urdf");
	robot = mWorld->getSkeleton("Krang");

	// grab the goal skeletons for visualization
	goal_skels[Krang::LEFT] = mWorld->getSkeleton("g1");
	goal_skels[Krang::RIGHT] = mWorld->getSkeleton("g2");

	// grab the end effectors so we can draw forces and velocities on them
	end_effs[Krang::LEFT] = robot->getNode("lGripper");
	end_effs[Krang::RIGHT] = robot->getNode("rGripper");

	// Initialize the somatic daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "krang-vis-tab";
	somatic_d_init(&daemon_cx, &dopt);

	// Start the daemon running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// start the main loop
	time_last = aa_tm_timespec2sec(aa_tm_now());
	timer = new KrangVisUpdateTimer();
	timer->Start(1e3 * (1.0 / DISPLAY_FREQUENCY));
}

/* ********************************************************************************************* */
/// Destructor, theoretically called when the tab is closing. We use this to close the hardware and
/// other ach channels if they're open.
KrangVisTab::~KrangVisTab() {
	// close the hardware
	if (checkbox_vis_krang_body->IsChecked()) {
		delete hw;
	}
	if (checkbox_vis_05_workspace_teleop->IsChecked()) {
		somatic_d_channel_close(&daemon_cx, &vis_chan);
	}

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Clean up the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
/// Called when the vis_krang_body checkbox becomes checked or unchecked. When this happens, we set
/// up or tear down ach channels and associated objects that we use for visulizing krang. We do
/// this at checkbox time so that we can start the application with only a few channels available
/// and turn on only those that we want to look at.
void KrangVisTab::OnCheckbox_vis_krang_body(wxCommandEvent& evt) {
	if (checkbox_vis_krang_body->IsChecked()) {
		// initialize and hook up the hardware object that we use to observe general krang state
		Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL);
		hw = new Krang::Hardware(mode, &daemon_cx, robot);
	} else {
		// destroy and turn off the hardware object
		delete hw;
	}
}

/* ********************************************************************************************* */
/// Called when the vis_krang_05_workspace_teleop checkbox becomes checked or unchecked. When this
/// happens, we set up or tear down ach channels and associated objects that we use for visulizing
/// krang. We do this at checkbox time so that we can start the application with only a few
/// channels available and turn on only those that we want to look at.
void KrangVisTab::OnCheckbox_vis_05_workspace_teleop(wxCommandEvent& evt) {
	if (checkbox_vis_05_workspace_teleop->IsChecked()) {
		// turn on all the stuff for visualizing 05 workspace teleop
		somatic_d_channel_open(&daemon_cx, &vis_chan, "teleop-05-workspace-vis", NULL);
	} else {
		// turn off all the stuff for visualizing 05 workspace teleop
		somatic_d_channel_close(&daemon_cx, &vis_chan);
	}
}

/* ********************************************************************************************* */
/// Called regularly by the timer object so that it happens DISPLAY_FREQUENCY times per second. We
/// use this function to update and visulize state; it's basically the inside of our main loop.
void KrangVisUpdateTimer::Notify() {
	// Update times
	double time_now = aa_tm_timespec2sec(aa_tm_now());
	double time_delta = time_now - time_last;
	time_last = time_now;

	// we update the hardware on our own clock to maintain consistency
	if(checkbox_vis_krang_body->IsChecked()) {
		// Update the robot
		hw->updateSensors(time_delta);
	}

	// and tell grip to update its rendering of the world
	viewer->DrawGLScene();
}

/* ********************************************************************************************* */
// If we're visualizing anything, display it
void KrangVisTab::GRIPEventRender() {
	// util variablese
	int ret;
	ach_status_t rach;
	size_t recv_bytes;

	// since the visualization data isn't doing anything with dynamics, we just grab it as it comes
	if(checkbox_vis_05_workspace_teleop->IsChecked()) {
		// try to grab a new visualization message
		Somatic__VisualizeData* vis_msg = SOMATIC_GET_LAST_UNPACK(rach, somatic__visualize_data,
		                                                          &protobuf_c_system_allocator, 4096, &vis_chan);
		// if we got one, throw out the old one and store the new one
		if (vis_msg != NULL) {
			if (vis_msg_cached != NULL)
				somatic__visualize_data__free_unpacked(vis_msg_cached, &protobuf_c_system_allocator);
			vis_msg_cached = vis_msg;
		}
		
		// either way, we almost certainly have something we can use, so use it!
		if (vis_msg_cached != NULL) {
			for(int sint = Krang::LEFT; sint-1 < Krang::RIGHT; sint++) {
				// Get the side
				Krang::Side sde = static_cast<Krang::Side>(sint);
				std::cout << "Side: " << (sde == Krang::LEFT ? "LEFT " : "RIGHT") << std::endl;

				// unpack the position reference and copy it over to the goal skeleton
				Krang::Vector6d pos_ref(vis_msg_cached->vecs[0 + (4 * sint)]->data);
				goal_skels[sde]->setConfig(Krang::dart_root_dof_ids, pos_ref);

				// grab the current end-effector position so we can draw arrows from it
				Eigen::Vector3d ee_pos_linear = end_effs[sde]->getWorldTransform().topRightCorner<3,1>();

				// draw an arrow showing the last estimation of the external force
				Krang::Vector6d ft_external(vis_msg_cached->vecs[3 + (4 * sint)]->data);
				Eigen::Vector3d ft_external_linear = ft_external.topRightCorner<3,1>();
				glColor4d(1.0, 0, 0, 0.5); // red
				yui::drawArrow3D(ee_pos_linear, ft_external_linear,
				                 ft_external_linear.norm() / 300.0, .01, .02);

				// draw an arrow showing the last ordered spacenav velocity
				Krang::Vector6d xdot_spacenav(vis_msg_cached->vecs[1 + (4 * sint)]->data);
				Eigen::Vector3d xdot_spacenav_linear = xdot_spacenav.topRightCorner<3,1>();
				glColor4d(0, 1.0, 0, 0.5); // green
				yui::drawArrow3D(ee_pos_linear, xdot_spacenav_linear,
				                 xdot_spacenav_linear.norm(), .01, .02);

				DISPLAY_VECTOR(pos_ref);
				DISPLAY_VECTOR(ee_pos_linear);
				DISPLAY_VECTOR(xdot_spacenav_linear);
			}
		}
	}
}


/* ############################################################################################# */
/* ############################################################################################# */
// build up the tab itself

/* ********************************************************************************************* */
// build event table
BEGIN_EVENT_TABLE(KrangVisTab, wxPanel)
EVT_CHECKBOX(event_checkbox_vis_krang_body, KrangVisTab::OnCheckbox_vis_krang_body)
EVT_CHECKBOX(event_checkbox_vis_05_workspace_teleop, KrangVisTab::OnCheckbox_vis_05_workspace_teleop)
END_EVENT_TABLE()

/* ********************************************************************************************* */
// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(KrangVisTab, GRIPTab)

/* ********************************************************************************************* */
// Necessary interface call to create a GRIP executable

/// Application class
extern wxNotebook* tabView;
class mainApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new KrangVisTab(tabView), wxT("Krang Vis"));
	}
};

IMPLEMENT_APP(mainApp)
