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
 * @file workspace.h
 * @author Can Erdogan, Saul Reynolds-Haertle
 * @date July 27, 2013
 * @brief This header file for workspace control of 7-dof arms.
 */

#pragma once

#include "util.h"

namespace Krang {

/// The interface workspace control - most importatnly contains the reference position for the
/// end-effector, and the input nullspace and damping gains
class WorkspaceControl {
public:


	/// Constructor
	WorkspaceControl (dynamics::SkeletonDynamics* robot, Side side, double _K_posRef_p, 
			double nullspace_gain, double damping_gain, double ui_translation_gain, 
			double ui_orientation_gain, double compliance_translation_gain, double compliance_orientation_gain);

	/// Integrates the input workspace velocity if one is given. This is for a user interface device
	/// such as spacenav or joystick whose input is more natural to interpret as velocities.
	void integrateWSVelocityInput(const VectorXd& xdot, const double dt);

	/// Returns a reference workspace velocity towards the integrated reference configuration from the
	/// current end-effector configuration
	void refWSVelocity(VectorXd& xdot);

	/// Returns a reference jointspace velocity from the given workspace velocity, biasing towards the
	/// the given jointspace velocity
	void refJSVelocity(const VectorXd& xdot, const VectorXd& qdot_nullspace, VectorXd& qdot);

	/// Returns the reference jointspace velocity incorporating the ui device and f/t sensor values
	void updateFromUIVel(const VectorXd& ui, const VectorXd& ft, const VectorXd& qdot_secondary, double dt, 
			VectorXd& qdot);

	/// Returns the reference jointspace velocity incorporating the ui device and f/t sensor values
	void updateFromXdot (const VectorXd& xdot, const VectorXd& ft, const VectorXd& qdot_secondary, double dt, 
			VectorXd& qdot);

	// Returns the reference jointspace velocity incorporating
	// position input from teh ui device and f/t sensor values
	void updateFromUIPos(const MatrixXd& xref, const VectorXd& ft, const VectorXd& qdot_secondary, VectorXd& qdot);

	/// Sets the workspace controller's goal to the current end
	/// effector position
	void resetReferenceTransform();

	/// Transforms a velocity-space user interface inptu into a
	/// workspace velocity
	Eigen::VectorXd uiInputVelToXdot(const VectorXd& ui_vel);

public:
	// Variables that represent the state of the end-effector or how we can control it

	Matrix4d Tref;														///< The integrated or set configuration reference
	kinematics::BodyNode* endEffector;				///< The end-effector whose configuration we control
	std::vector<int>* arm_ids;								///< The arm indices that the controller can manipulate
	bool debug_to_cout;							///< Verbosity for printing debug to standard output
	bool debug_to_curses;							///< Verbosity for printing debug to curses
	
public:
	// The gains that affect the control

	double K_posRef_p;			///< The error gain for the P-controller in deducing ref. WS. velocities
	double nullspace_gain;	///< The gain which affects how much the reference JS. vel are biased
	double damping_gain;		///< The damping factor on the Jacobian to deal with singularities
	double ui_translation_gain;		///< The constant multiplier for ui inputs translations (vel/pos)
	double ui_orientation_gain;		///< The constant multiplier for ui inputs orientations (vel/pos)
	double compliance_translation_gain;	///< The effect of compliance in the workspace control
	double compliance_orientation_gain;	///< The effect of compliance in the workspace control

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};	// end of namespace

// Local Variables:
// mode: c++
// End:
