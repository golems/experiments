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

#include "simTab.h"
#include "GRIPApp.h"

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <math/UtilsRotation.h>

using namespace std;
using namespace Eigen;
using namespace dynamics;

/* ********************************************************************************************* */

// Channel vairables
uint8_t *achbuf;
size_t n_achbuf = 1024;
size_t frame_size;
size_t indent = 0;

somatic_d_t daemon_cx;
ach_channel_t liberty_chan;

void ach_init(ach_channel_t* chan, char* chan_name, size_t n_achbuf) {
    // Set up the buffer
    achbuf = AA_NEW_AR(uint8_t,  n_achbuf );
    
    // Open the given channel 
    int r = ach_open( chan, chan_name, NULL );
    aa_hard_assert( ACH_OK == r, "Couldn't open channel %s\n", chan_name );
    r = ach_flush( chan );
    aa_hard_assert( ACH_OK == r, "Couldn't flush channel\n");
    
    // Set the interrupt handler
    somatic_sighandler_simple_install();
}

/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left 
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {

    // get liberty data
    int r = 0;
    Somatic__Liberty *ls_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__liberty, 
							 &protobuf_c_system_allocator, 
	4096, &liberty_chan );
    if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (ls_msg == NULL)) return;

    
    // stack sensors into a single array for indexing
    Somatic__Vector* sensors[] = {ls_msg->sensor1, ls_msg->sensor2, ls_msg->sensor3, ls_msg->sensor4,
				  ls_msg->sensor5, ls_msg->sensor6, ls_msg->sensor7, ls_msg->sensor8};
    
    // pack liberty data into arrowConfs
    for (int i=0; i < 8; ++i) {
	Somatic__Vector* sensor = sensors[i];
	VectorXd arrowConf (6);
	
	for (int j=0; j < 3; j++) 
	    arrowConf[j] = sensor->data[j];
	
	// convert quat to euler
	Eigen::Quaternion<double> rotQ(&sensor->data[3]);
	Matrix3d rotM(rotQ);
	Vector3d rotV = math::matrixToEuler(rotM, math::XYZ);
	
	for (int j=3; j < 6; j++) 
	    arrowConf[j] = rotV[j-3];

	vector <int> conf_ids;
	for(size_t i = 0; i < 6; i++) conf_ids.push_back(i);
	mWorld->getSkeleton(i)->setConfig(conf_ids, arrowConf);
    }

    // Free the liberty message
    somatic__liberty__free_unpacked(ls_msg, &protobuf_c_system_allocator);

    // Restart the timer for the next start
    viewer->DrawGLScene();
    Start(0.005 * 1e4);	
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
	       long style) : GRIPTab(parent, id, pos, size, style) {

    // ============================================================================
    // Initialize grip stuff

    sizerFull = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
    viewer->camRadius = 3.0;
    viewer->worldV += Vector3d(0.0, 0.0, -0.7);
    viewer->UpdateCamera();
    SetSizer(sizerFull);
    frame->DoLoad("../../common/scenes/04-World-Liberty.urdf");

    // Create the timer to notify the function that draws the robot at multiple configurations
    timer = new Timer();
    timer->Start(0.001);	

    // ============================================================================
    // Initialize this daemon (program!)
    somatic_d_opts_t dopt;
    memset(&dopt, 0, sizeof(dopt)); // zero initialize
    dopt.ident = "02-liberty-display";
    somatic_d_init(&daemon_cx, &dopt);

    // Initialize the ach channel for liberty
    ach_init(&liberty_chan, "liberty", n_achbuf);

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
// Handler for events

BEGIN_EVENT_TABLE(SimTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimTab::OnSlider)
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
