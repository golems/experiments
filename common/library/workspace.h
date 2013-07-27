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
			double ui_orientation_gain, double compliance_gain);

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
	void update (const VectorXd& ui, const VectorXd& ft, const VectorXd& qdot_secondary, double dt, 
			VectorXd& qdot);

public:
	// Variables that represent the state of the end-effector or how we can control it

	Matrix4d Tref;														///< The integrated or set configuration reference
	kinematics::BodyNode* endEffector;				///< The end-effector whose configuration we control
	std::vector<int>* arm_ids;								///< The arm indices that the controller can manipulate
	bool debug;							///< Debug verbosity
	
public:
	// The gains that affect the control

	double K_posRef_p;			///< The error gain for the P-controller in deducing ref. WS. velocities
	double nullspace_gain;	///< The gain which affects how much the reference JS. vel are biased
	double damping_gain;		///< The damping factor on the Jacobian to deal with singularities
	double ui_translation_gain;		///< The constant multiplier for ui inputs translations (vel/pos)
	double ui_orientation_gain;		///< The constant multiplier for ui inputs orientations (vel/pos)
	double compliance_gain;	///< The effect of compliance in the workspace control
};

};	// end of namespace
