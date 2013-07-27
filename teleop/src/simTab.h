/**
 * @file ik-simTab.h
 * @author Can Erdogan
 * @date May 03, 2013
 * @brief Simulates the inverse kinematics behavior of the robot. When the arm is taken to a 
 * location and the location is saved, an inverse kinematics solution to the arm is found and
 * presented.
 */

#pragma once

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>
#include <Tools/Constants.h>
#include <wx/wx.h>
#include <wx/statbox.h>
#include <wx/tglbtn.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/AllTabs.h>
#include <GRIPApp.h>
#include <collision/fcl_mesh/CollisionShapes.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <simulation/World.h>

/* ********************************************************************************************* */
/// Timer to display the center of mass measurements and control the robot
class Timer : public wxTimer {
public:
	void Notify ();								
};

/* ********************************************************************************************* */
/// Tab for examples of task constrained planning
class SimTab : public GRIPTab {
public:
	Timer* timer;

	SimTab(){};									///< Default constructor
	SimTab(wxWindow * parent, wxWindowID id = -1, const wxPoint & pos = wxDefaultPosition,
			const wxSize & size = wxDefaultSize, long style = wxTAB_TRAVERSAL);
	virtual ~SimTab();				///< Destructor
	void OnButton(wxCommandEvent &evt);		///< Handle button events
	void OnSlider(wxCommandEvent &evt);		///< Handle slider events
	virtual void GRIPEventSimulationBeforeTimestep();  ///< To set joint torques before sim. step

public:
	// wxWidget stuff

	DECLARE_DYNAMIC_CLASS(SimTab)
DECLARE_EVENT_TABLE()
};

