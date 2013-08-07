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

#include "simTab.h"
#include "GRIPApp.h"
#include <math/UtilsRotation.h>

#include "workspace.hpp"
#include "sensors.hpp"
#include "safety.hpp"
#include "display.hpp"

somatic_d_t daemon_cx;
dynamics::SkeletonDynamics* robot;
std::map<Krang::Side, Krang::Vector7d> nullspace_qdot_refs;			///< nullspace configurations for the arms

std::map<Krang::Side, Krang::SpaceNav*> spnavs; ///< points to spacenavs

// workspace stuff
std::map<Krang::Side, Krang::WorkspaceControl*> wss; ///< does workspace control for the arms

// sync-mode stuff
Eigen::MatrixXd Trel_left_to_right; ///< translation from the left hand to the right hand
Eigen::MatrixXd Trel_mid_to_right; ///< translation from the manipulation middle to the right hand
Eigen::MatrixXd Trel_mid_to_left; ///< translation from the manipulation middle to the left hand

/* ********************************************************************************************* */
/// Configurations for workspace control
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.75;
const double SPACENAV_TRANSLATION_GAIN = 0.25; 
const double COMPLIANCE_TRANSLATION_GAIN = 1.0 / 750.0;
const double COMPLIANCE_ORIENTATION_GAIN = .125 / 750.0;

const double LOOP_FREQUENCY = 10.0;
const double DISPLAY_FREQUENCY = 3.0;

bool debug_print_this_it;       ///< whether we print

/* ********************************************************************************************* */
/// Gets the workspace velocity input from the spacenav, updates a workspace reference position
/// and moves the left arm end-effector to that position
void Timer::Notify() {

	// ============================================================================
	// Handle some timing and debug stuff

	// Update times
	static double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	static double time_last = aa_tm_timespec2sec(aa_tm_now());
	double time_now = aa_tm_timespec2sec(aa_tm_now());
	double time_delta = time_now - time_last;
	time_last = time_now;

	// set up debug prints
	debug_print_this_it = (time_now - time_last_display) > (1.0 / DISPLAY_FREQUENCY);
	if (debug_print_this_it) time_last_display = time_now;
	// wss[Krang::LEFT]->debug = debug_print_this_it;
	// wss[Krang::RIGHT]->debug = debug_print_this_it;

	if(debug_print_this_it) 
		std::cout << "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << std::endl;

	// ================================================================================
	// elbows
	for(int i = Krang::LEFT; i-1 < Krang::RIGHT; i++) {
		Krang::Side sde = static_cast<Krang::Side>(i);
		
		// Get the joint angles we want to bias towards
		Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

		VectorXd q_elbow_ref = q;
		q_elbow_ref(3) = sde == Krang::LEFT ? -0.5 : 0.5;
		nullspace_qdot_refs[sde] = (q_elbow_ref - q).normalized();
		if (debug_print_this_it)
			DISPLAY_VECTOR(nullspace_qdot_refs[sde]);
	}

	// ================================================================================
	// left arm
	
	// Get a workspace velocity input from the spacenav
	Eigen::VectorXd spacenav_input_L = spnavs[Krang::LEFT]->updateSpaceNav();
	Eigen::VectorXd xdot_spacenav_L = wss[Krang::LEFT]->uiInputVelToXdot(spacenav_input_L);

	// compute the desired jointspace velocity from the inputs (ft is zero)
	Eigen::VectorXd qdot_jacobian_L;
	VectorXd ft_L = VectorXd::Zero(6);
	wss[Krang::LEFT]->updateFromXdot(xdot_spacenav_L, ft_L, nullspace_qdot_refs[Krang::LEFT], time_delta, qdot_jacobian_L);
	
	// make sure we're not going too fast
	double magn = qdot_jacobian_L.norm();
	if (magn > 0.5) qdot_jacobian_L *= (0.5 / magn);
	if (magn < .05) qdot_jacobian_L *= 0.0;
	
	// avoid joint limits
	Eigen::VectorXd q_left = robot->getConfig(*wss[Krang::LEFT]->arm_ids);
	Eigen::VectorXd qdot_avoid_L(7);
	Krang::computeQdotAvoidLimits(robot, *wss[Krang::LEFT]->arm_ids, q_left, qdot_avoid_L);
	
	// add qdots together to get the overall movement
	Eigen::VectorXd qdot_apply_L = qdot_avoid_L + qdot_jacobian_L;

	// and apply that to the right arm
	q_left += qdot_apply_L * time_delta;
	robot->setConfig(*wss[Krang::LEFT]->arm_ids, q_left);

	// ================================================================================
	// right arm
	
	// figure out where the position reference should be using the
	// relative transofrmation from the left hand to the right hand
	Eigen::MatrixXd Tref_R = wss[Krang::LEFT]->Tref * Trel_left_to_right;

	// update the workspace controller for the other arm
	Eigen::VectorXd qdot_jacobian_R;
	Eigen::VectorXd ft_R = VectorXd::Zero(6);
	wss[Krang::RIGHT]->updateFromUIPos(Tref_R, ft_R, nullspace_qdot_refs[Krang::RIGHT], qdot_jacobian_R);

	// make sure we're not going too fast
	magn = qdot_jacobian_R.norm();
	if (magn > 0.5) qdot_jacobian_R *= (0.5 / magn);
	if (magn < .05) qdot_jacobian_R *= 0.0;
	
	// avoid joint limits
	Eigen::VectorXd qdot_avoid_R(7);
	Eigen::VectorXd q_right = robot->getConfig(*wss[Krang::RIGHT]->arm_ids);
	Krang::computeQdotAvoidLimits(robot, *wss[Krang::RIGHT]->arm_ids, q_right, qdot_avoid_R);

	// add qdots together to get the overall movement
	Eigen::VectorXd qdot_apply_R = qdot_avoid_R + qdot_jacobian_R;
	
	// and apply that to the right arm
	q_right += qdot_apply_R * time_delta;
	robot->setConfig(*wss[Krang::RIGHT]->arm_ids, q_right);

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
	if (debug_print_this_it)
		std::cout << "time: " << time_loop_end - time_now << "/" << (1.0/LOOP_FREQUENCY) << std::endl;
	Start(time_sleep_msecs);
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size,
               long style) : GRIPTab(parent, id, pos, size, style) {

	// ============================================================================
	// Initialize grip stuff

	wxBoxSizer* sizerFull = new wxBoxSizer(wxHORIZONTAL);
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(-0.3, 0.0, -0.8);
	Matrix3d rotM; 
	rotM << -0.8459, 0.0038246, 0.5332,0.000573691,-0.9999,0.008082, 0.533265, 0.00714295, 0.845918;
	viewer->camRotT = rotM;
	viewer->UpdateCamera();
	SetSizer(sizerFull);
	frame->DoLoad("../../common/scenes/05-World-Teleop.urdf");
	robot = mWorld->getSkeleton("Krang");

	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->Start(1);	

	// ============================================================================
	// Initialize robot stuff

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
	                                               SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN, COMPLIANCE_TRANSLATION_GAIN,
	                                               COMPLIANCE_ORIENTATION_GAIN);
	wss[Krang::RIGHT] = new Krang::WorkspaceControl(robot, Krang::RIGHT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
	                                                SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN, COMPLIANCE_TRANSLATION_GAIN,
	                                                COMPLIANCE_ORIENTATION_GAIN);

	// Manually set the initial arm configuration for the arms
	Eigen::VectorXd homeConfigsL(7);
	homeConfigsL <<  0.97, -0.589,  0.000, -1.339,  0.000, -0.959, -1.000;
	Eigen::VectorXd homeConfigsR(7);
	homeConfigsR << -1.102,  0.589,  0.000,  1.339,  0.141,  0.959, -1.000;
	robot->setConfig(*wss[Krang::LEFT]->arm_ids, homeConfigsL);
	robot->setConfig(*wss[Krang::RIGHT]->arm_ids, homeConfigsR);
	wss[Krang::LEFT]->resetReferenceTransform();
	wss[Krang::RIGHT]->resetReferenceTransform();

	// set up the relative transform between the hands
	Trel_left_to_right = wss[Krang::LEFT]->Tref.inverse() * wss[Krang::RIGHT]->Tref;

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ********************************************************************************************* */
SimTab::~SimTab() {

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
void SimTab::GRIPEventSimulationBeforeTimestep() {}
void SimTab::OnButton(wxCommandEvent &evt) {}
void SimTab::OnSlider(wxCommandEvent &evt) {}

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
		tabView->AddPage(new SimTab(tabView), wxT("Teleop"));
	}
};

IMPLEMENT_APP(mainApp)


