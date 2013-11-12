/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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
 * @file 01-penetration-depth.cpp
 * @author Saul Reynolds-Haertle
 * @date November 12, 2013

 * @brief This executable demonstrates extracting and displaying the depth of penetration between
 * two colliding bodies, particularly two colliding bodies on the same skeleton
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
#include <collision/CollisionDetector.h>

// wxwidgets stuff
#include <wx/wx.h>
#include <wx/statbox.h>
#include <wx/tglbtn.h>

// general stuff stuff
#include <Eigen/Dense>

// kore
#include <kore.hpp>
#include <kore/util.hpp>


/* ############################################################################################### */
/// Declare the class for our tab
class PenetrationDepthTab : public GRIPTab {
public:
	PenetrationDepthTab(){};									///< Default constructor
	PenetrationDepthTab(wxWindow* parent, wxWindowID id = -1, const wxPoint& pos = wxDefaultPosition,
	            const wxSize& size = wxDefaultSize, long style = wxTAB_TRAVERSAL);
	virtual ~PenetrationDepthTab();				///< Destructor

    virtual void GRIPEventRender();
	
public:
	DECLARE_DYNAMIC_CLASS(SimTab)
	DECLARE_EVENT_TABLE()
};

/* ############################################################################################### */
/// Constants

// ui event ids
enum PenetrationDepthTabEvents {
};

/* ############################################################################################### */
/// Variables

// pointers to important DART objects
dynamics::SkeletonDynamics* robot;

/* ############################################################################################### */
/// constructor

PenetrationDepthTab::PenetrationDepthTab(wxWindow *parent,
                                         const wxWindowID id,
                                         const wxPoint& pos,
                                         const wxSize& size,
                                         long style)
	: GRIPTab(parent, id, pos, size, style) {

	// create layout objects
	wxBoxSizer* tab_sizer = new wxBoxSizer(wxHORIZONTAL);
	
	// organize layout objects

	// create actual UI elements

	// lay out actual UI elements
	
	// and finalize the layout
	SetSizer(tab_sizer);

	// set the initial camera angle
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(-0.3, 0.0, -0.8);
	Matrix3d rotM; 
	rotM << -0.8459, 0.0038246, 0.5332,0.000573691,-0.9999,0.008082, 0.533265, 0.00714295, 0.845918;
	viewer->camRotT = rotM;
	viewer->UpdateCamera();

	// load up DART
	frame->DoLoad("../../common/scenes/01-World-Robot.urdf");
	robot = mWorld->getSkeleton("Krang");
	robot->setSelfCollidable(false);
	mWorld->reset();

	// set the imu and waist angles
	std::vector <int> imuWaist_ids;
	imuWaist_ids.push_back(5);
	imuWaist_ids.push_back(8);
	Vector2d imuWaist (3.45, 2.81);
	robot->setConfig(imuWaist_ids, imuWaist);

	// Manually set the initial arm configuration for the arms
	Eigen::VectorXd homeConfigsL(7);
	homeConfigsL <<  0.97, -0.589,  0.000, -1.339,  0.000, -0.959, -1.000;
	Eigen::VectorXd homeConfigsR(7);
	homeConfigsR << -1.102,  0.589,  0.000,  1.339,  0.141,  0.959, -1.000;
	robot->setConfig(Krang::left_arm_ids, homeConfigsL);
	robot->setConfig(Krang::right_arm_ids, homeConfigsR);
}

/* ############################################################################################### */
PenetrationDepthTab::~PenetrationDepthTab() {
}

/* ############################################################################################### */
void PenetrationDepthTab::GRIPEventRender() {
	// set up opengl for rendering lines to show contacts
    glDisable(GL_FOG);
    glEnable(GL_COLOR_MATERIAL);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    glLineWidth(1.5f);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);

    // find our contacts
	dynamics::ContactDynamics* collider = mWorld->getCollisionHandle();
	collision::CollisionDetector* checker = collider->getCollisionChecker();
	bool collided = checker->checkCollision(true, true);
	int numcontacts = checker->getNumContacts();
	std::cout << "collided: " << collided << ", num contacts: " << numcontacts << std::endl;

	// and actually do the rendering
	Eigen::Vector3d v;
	Eigen::Vector3d f;
	Eigen::Vector3d vf;
	Eigen::Vector3d arrowheadDir;
	Eigen::Vector3d arrowheadBase;
	glBegin(GL_LINES);
	for(int contactidx = 0; contactidx < numcontacts; contactidx++) {
		collision::Contact& cont = checker->getContact(contactidx);
		glColor3d(0.0, 1.0, 0.0);
		v = cont.point;
		f = cont.normal.normalized() * .1;
		vf = v + f;
		arrowheadDir = v.cross(f).normalized() * .0075;
		arrowheadBase = vf - f.normalized() * .02;
		glVertex3f(v[0], v[1], v[2]);
		glVertex3f(vf[0], vf[1], vf[2]);
		glVertex3f(vf[0], vf[1], vf[2]);
		glVertex3f(arrowheadBase[0] + arrowheadDir[0], arrowheadBase[1] + arrowheadDir[1], arrowheadBase[2] + arrowheadDir[2]);
		glVertex3f(vf[0], vf[1], vf[2]);
		glVertex3f(arrowheadBase[0] - arrowheadDir[0], arrowheadBase[1] - arrowheadDir[1], arrowheadBase[2] - arrowheadDir[2]);
	}
	glEnd();
}

/* ############################################################################################### */
// event table

BEGIN_EVENT_TABLE(PenetrationDepthTab, wxPanel)
END_EVENT_TABLE()

/* ############################################################################################### */
// Class constructor for the tab: Each tab will be a subclass of GRIPTab

IMPLEMENT_DYNAMIC_CLASS(PenetrationDepthTab, GRIPTab)

/* ############################################################################################### */
// Necessary interface call to create a GRIP executable

/// Application class
extern wxNotebook* tabView;
class mainApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new PenetrationDepthTab(tabView), wxT("penetration depth"));
	}
};

IMPLEMENT_APP(mainApp)


