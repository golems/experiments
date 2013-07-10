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
// Definitions for the left and right arm indices
int left_idx_a [] = {10, 12, 14, 16, 18, 20, 22};
vector <int> left_idx (left_idx_a, left_idx_a + sizeof(left_idx_a) / sizeof(int));
int right_idx_a [] = {11, 13, 15, 17, 19, 21, 23};
vector <int> right_idx (right_idx_a, right_idx_a + sizeof(right_idx_a) / sizeof(int));

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

  wxSizer* sizerFull;											///< The sizer in charge of the entire frame
	Timer* timer;

public:
	// Mandatory interface functions

  SimTab(){};									///< Default constructor
  SimTab(wxWindow * parent, wxWindowID id = -1, const wxPoint & pos = wxDefaultPosition,
		const wxSize & size = wxDefaultSize, long style = wxTAB_TRAVERSAL);		
  virtual ~SimTab();				///< Destructor
  void OnButton(wxCommandEvent &evt) {}		///< Handle button events
  void OnSlider(wxCommandEvent &evt) {}		///< Necessary for compilation (bug!)
  virtual void GRIPEventSimulationBeforeTimestep();  ///< To set joint torques before sim. step

	/// Prepare the data structures of frame for a video
	void prepareVideo () {
	}

public:
	// wxWidget stuff

  DECLARE_DYNAMIC_CLASS(SimTab)
	DECLARE_EVENT_TABLE()
};

