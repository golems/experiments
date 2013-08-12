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
 * @file krang-vis-tab.hpp
 * @author Saul Reynolds-Haertle
 * @date Aug 05, 2013
 * @brief This tab visualizes the current state of krang, including (if that data is available) the
 * internal state of the examples which publish that information for visualization.
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
/// Timer so that we're updating state at a reasonable frequency
class KrangVisUpdateTimer : public wxTimer {
public:
	void Notify ();
};


/* ********************************************************************************************* */
/// Tab for visualizing krang's state
class KrangVisTab : public GRIPTab {
public:
	KrangVisUpdateTimer* timer;

	KrangVisTab(){};									///< Default constructor
	KrangVisTab(wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
	            const wxSize& size = wxDefaultSize, long style = wxTAB_TRAVERSAL);
	virtual ~KrangVisTab();				///< Destructor

    // *************************************
	// GRIP event handlers
	virtual void GRIPEventRender(); ///< Called when GRIP is rendering - do visalization in here
    // *************************************  

	/// Visualizes general krang state, things like the waist and arm angles and the IMU angle
	void OnCheckbox_vis_krang_body(wxCommandEvent& evt);

	/// Visualizes internal state from the teleop/05-workspace example, most importantly the
	/// position references used by that example to mix together velocity inputs and compliance.
	void OnCheckbox_vis_05_workspace_teleop(wxCommandEvent& evt);

public:
	// wxWidget stuff

	DECLARE_DYNAMIC_CLASS(SimTab)
	DECLARE_EVENT_TABLE()
};

