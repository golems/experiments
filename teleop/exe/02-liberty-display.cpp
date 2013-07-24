/**
 * @file 02-liberty-display.cpp
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

#define pv(a) std::cout << #a << ": " << fix((a).transpose()) << std::endl

/* ******************************************************************************************** */
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < 1e-5) mat2(i,j) = 0.0;
	return mat2;
}

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t liberty_chan;

/* ********************************************************************************************* */
/// Returns the values of the first liberty sensor
bool getLiberty(VectorXd& config) {

	// Get the data
	int r = 0;
	config.setZero(); 
	Somatic__Liberty *l_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__liberty,
			&protobuf_c_system_allocator, 1024, &liberty_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (l_msg == NULL)) return false;

	// Set the values for the position
	Somatic__Vector* s1 = l_msg->sensor1;
	config << s1->data[0], -s1->data[1], -s1->data[2], 0.0, 0.0, 0.0;

	// Convert from a quaternion to rpy representation
	Quaternion <double> oriQ (s1->data[6], s1->data[3], s1->data[4], s1->data[5]);
	Matrix3d oriM = oriQ.matrix();
	Vector3d oriE = math::matrixToEuler(oriM, math::XYZ);
	config.bottomLeftCorner<3,1>() << -oriE[2], -oriE[1], oriE[0];
	return true;
}

/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left 
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {
	
	// Get the liberty info
	VectorXd liberty_input (6);
	bool result = false;
	while(!result) result = getLiberty(liberty_input);
	pv(liberty_input);

//	std::vector<Matrix4d> poses = liberty_rel_mode ? liberty.getRelPoses() : liberty.getRawPoses();
//	for (int i=0; i < 8; ++i) {
//		cout << poses[i] << endl;
//		VectorXd arrowConf = transformToEuler(poses[i], math::XYZ);
//		mWorld->getSkeleton(i)->setConfig(dartRootDofOrdering, arrowConf);
//	}

	// Restart the timer for the next start
	viewer->DrawGLScene();
	Start(0.001 * 1e4);
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
				 long style) : GRIPTab(parent, id, pos, size, style) {

	// ============================================================================
	// Create user interface
	wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(-0.3, 0.0, -0.8);
	Matrix3d rotM; 
	rotM << -0.8459, 0.0038246, 0.5332,0.000573691,-0.9999,0.008082, 0.533265, 0.00714295, 0.845918;
	viewer->camRotT = rotM;
	SetSizer(sizerFull);
	frame->DoLoad("../../common/scenes/05-World-Teleop.urdf");

	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->Start(1);

	// ============================================================================
	// Initialize the ach stuff

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "02-liberty-display";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the liberty client
	somatic_d_channel_open(&daemon_cx, &liberty_chan, "liberty", NULL);
}

/* ********************************************************************************************* */
SimTab::~SimTab() {
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
