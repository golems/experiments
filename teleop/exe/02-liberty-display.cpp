/**
 * @file 04-gripVis.cpp
 * @author Can Erdogan
 * @date June 18, 2013
 * @brief This file demonstrates how to visualize the motion of the arms in grip.
 * NOTE Although I wanted to change this, we had to make the GRIP/wxWidget the main program
 * (the surrounding thread) and send data within a timer... This could be bad if for some reason
 * visualization halts and we want to stop the arms right then.
 */


#define protected public
#define private public


#include <GRIPApp.h>
#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <math/UtilsRotation.h>

#include "simTab.h"
#include "util.h"
#include "LibertyClient.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;

/* ********************************************************************************************* */

// somatic daemon
somatic_d_t daemon_cx;

// Liberty client
LibertyClient liberty;

enum DynamicSimulationTabEvents {
	id_button_ResetLiberty = 8100,
	id_checkbox_ToggleLibertyRelMode,
};

static bool liberty_rel_mode = 0;

/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left 
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {
	
	std::vector<Matrix4d> poses = liberty_rel_mode ? liberty.getRelPoses() : liberty.getRawPoses();
	for (int i=0; i < 8; ++i) {
		cout << poses[i] << endl;
		VectorXd arrowConf = transformToEuler(poses[i], math::XYZ);
		mWorld->getSkeleton(i)->setConfig(dartRootDofOrdering, arrowConf);
	}

    // Restart the timer for the next start
    viewer->DrawGLScene();
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
	       long style) : GRIPTab(parent, id, pos, size, style) {

	// Create user interface
	wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("UI Input"));
	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	ss1BoxS->Add(new wxButton(this, id_button_ResetLiberty, wxT("Set Liberty Initial Transforms")), 0, wxALL, 1);
	ss1BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleLibertyRelMode, wxT("Use Liberty Relative Poses")), 0, wxALL, 1);
	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	SetSizer(sizerFull);

    // ============================================================================
    // Initialize grip stuff
    viewer->camRadius = 3.0;
    viewer->worldV += Vector3d(0.0, 0.0, -0.7);
    viewer->UpdateCamera();
    frame->DoLoad("../../common/scenes/04-World-Liberty.urdf");
	if (mWorld == NULL)
		frame->DoLoad("common/scenes/04-World-Liberty.urdf"); // for eclipse

    // Create the timer to notify the function that draws the robot at multiple configurations
    timer = new Timer();
    timer->Start(0.005 * 1e4);

    // ============================================================================
    // Initialize this daemon (program!)
    somatic_d_opts_t dopt;
    memset(&dopt, 0, sizeof(dopt)); // zero initialize
    dopt.ident = "02-liberty-display";
    somatic_d_init(&daemon_cx, &dopt);

    // Initialize the liberty client
    int channels[] = {0,1,2,3,4,5,6,7};
    liberty.initLiberty(&daemon_cx, "liberty", 8, channels);

    // Send a message; set the event code and the priority
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
    SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

}

/* ********************************************************************************************* */
SimTab::~SimTab() {

    // Send the stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
		    SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // Clean up the daemon resources
    somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() {}

/* ********************************************************************************************* */
/**
 * @function OnButton
 * @brief Handles button events
 */
void SimTab::OnButton(wxCommandEvent &evt) {
	int slnum = evt.GetId();
	switch(slnum) {
	// Set Start Arm Configuration
	case id_button_ResetLiberty: {
		liberty.setInitialPoses();
		break;
	}

	case id_checkbox_ToggleLibertyRelMode: {
		liberty_rel_mode = evt.IsChecked();
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
EVT_CHECKBOX(id_checkbox_ToggleLibertyRelMode, SimTab::OnButton)
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
	tabView->AddPage(new SimTab(tabView), wxT("Liberty"));
    }
};

IMPLEMENT_APP(mainApp)
