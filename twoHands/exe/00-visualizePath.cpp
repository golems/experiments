/**
 * @file 08-simJInv.cpp
 * @author Can Erdogan
 * @date May 24, 2013
 * @brief This executable visualizes the path the robot follows in grip. 
 */

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include "simTab.h"

#define pv(a) std::cout << #a << ": " << (a).transpose() << std::endl
#define pmr(a) std::cout << #a << ":\n " << ((a)) << "\n" << std::endl

using namespace std;
using namespace Eigen;
using namespace dynamics;
using namespace simulation;

vector <VectorXd> path;           ///< The path that the robot will execute
vector <int> dofs;                ///< Indices to set the configuration

bool justVisualizeScene = 0;

void SimTab::GRIPEventSimulationBeforeTimestep() {

}
/* ********************************************************************************************* */
/// Visualizes the robot path
void Timer::Notify() {

	// Draw the skeleton
	static size_t path_idx = 0;
	mWorld->getSkeleton(1)->setConfig(dofs, path[path_idx]);	

	// Get the orientation
	kinematics::BodyNode* eeNode = world->getSkeleton(1)->getNode("lgPlate1");
	MatrixXd eeTransform = eeNode->getWorldTransform();
	double angle = atan2(eeTransform(1,3), eeTransform(0,3) - 0.44);
	vector <int> valve_dofs;
	valve_dofs.push_back(3);
	VectorXd bla (1);
	bla << angle;
	mWorld->getSkeleton(0)->setConfig(valve_dofs, bla);

	// Restart the timer for the next turn 
	viewer->DrawGLScene();
	path_idx = (path_idx + 1) % path.size();
	Start(0.05 * 1e3);	
}

/* ********************************************************************************************* */
void readFile () {

	// Open the file
	fstream file ("data");
	assert(file.is_open() && "Could not open the file!");

	// Read each line
	double temp;
	std::string line;
	VectorXd row = VectorXd::Zero(24);
	while(getline(file, line)) {
		size_t i = 0;
		stringstream stream (line, stringstream::in);
		while(stream >> temp) row(i++) = temp;
		path.push_back(row);
	}
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
		long style) : GRIPTab(parent, id, pos, size, style) {

	// Create the sizer
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
 	// Create the sizer that controls the tab panel
	wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
	
	// Set the camera zoom
	viewer->camRadius = 1.0;
	viewer->UpdateCamera();

	// Set the full sizer as the sizer of this tab
	SetSizer(sizerFull);

	// Load the schunk scene automatically
	frame->DoLoad("/home/cerdogan/Documents/MacGyver/3rdParty/simulation/scenes/08-World-Valve.urdf");
//	frame->DoLoad("/home/cerdogan/Documents/MacGyver/3rdParty/simulation/scenes/00-World-Test.urdf");

	// Return immediately if just want to see a scene and not a path
	if(justVisualizeScene) return;

	// Read the data file
	readFile();

	// Set the initial configuration
	SkeletonDynamics* robot = mWorld->getSkeleton(1);
	for(size_t i = 0; i < 24; i++) dofs.push_back(i);
	robot->setConfig(dofs, path[0]);

	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->world = mWorld;
	timer->Start(1);	
}

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
		tabView->AddPage(new SimTab(tabView), wxT("Inverse Kinematics"));
	}
};

IMPLEMENT_APP(mainApp)
