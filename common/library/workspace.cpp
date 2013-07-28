/**
 * @file workspace.cpp
 * @author Can Erdogan, Saul Reynolds-Haertle
 * @date July 27, 2013
 * @brief This header file for workspace control of 7-dof arms.
 */

#include "workspace.h"
#include <amino/math.h>

namespace Krang {

/* ******************************************************************************************** */
WorkspaceControl::WorkspaceControl (dynamics::SkeletonDynamics* robot, Side side, 
		double _K_posRef_p, double _nullspace_gain, double _damping_gain, 
		double _ui_translation_gain, double _ui_orientation_gain, double _compliance_gain) {

	// Determine the end-effector and the arm indices based on the input side
	endEffector = robot->getNode((side == LEFT) ? "lGripper" : "rGripper");
	arm_ids = (side == LEFT) ? (&left_arm_ids) : (&right_arm_ids);
	Tref = endEffector->getWorldTransform();
	
	// Set the gains for control
	K_posRef_p = _K_posRef_p, nullspace_gain = _nullspace_gain, damping_gain = _damping_gain;

	// Set the gains for sensors
	ui_translation_gain = _ui_translation_gain, ui_orientation_gain = _ui_orientation_gain;
	compliance_gain = _compliance_gain;
	pc(K_posRef_p);
	pc(nullspace_gain);
	pc(damping_gain);
	pc(ui_translation_gain);
	pc(ui_orientation_gain);
	pc(compliance_gain);
	debug = false;
}

/* ******************************************************************************************** */
	void WorkspaceControl::integrateWSVelocityInput(const VectorXd& xdot, const double dt) {

	// Represent the workspace velocity input as a 4x4 homogeneous matrix
	Matrix4d xdotM = eulerToTransform(xdot * dt, math::XYZ);
	
	// Compute the displacement in the end-effector frame with a similarity transform
	Matrix4d R = Tref;
	R.topRightCorner<3,1>().setZero();
	Matrix4d Tdisp = R.inverse() * xdotM * R;

	// Update the reference position for the end-effector with the given workspace velocity
	Tref = Tref * Tdisp;
}

/* ******************************************************************************************** */
void WorkspaceControl::refWSVelocity(VectorXd& xdot) {

	// Get the current end-effector transform and also, just its orientation 
	Matrix4d Tcur = endEffector->getWorldTransform();
	Matrix4d Rcur = Tcur;
	Rcur.topRightCorner<3,1>().setZero();

	// Apply the similarity transform to the displacement between current transform and reference
	Matrix4d Tdisp = Tcur.inverse() * Tref;
	Matrix4d xdotM = Rcur * Tdisp * Rcur.inverse();
	xdot = transformToEuler(xdotM, math::XYZ) * K_posRef_p;
}

/* ******************************************************************************************** */
void WorkspaceControl::refJSVelocity(const VectorXd& xdot, const VectorXd& qdot_nullspace, VectorXd& qdot) {

	// Get the Jacobian for the end-effector
	MatrixXd Jlin = endEffector->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = endEffector->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian with dampening
	MatrixXd Jt = J.transpose();
	MatrixXd JJt = J * Jt;
	for(int i = 0; i < JJt.rows(); i++) JJt(i,i) += damping_gain;
	
	// Compute the joint velocities qdot using the input xdot and a qdot for the secondary goal 
	// projected into the nullspace
	double aminoInv [36];
	for(size_t i = 0, c = 0; i < 6; i++)
		for(size_t j = 0; j < 6; j++, c++)
			aminoInv[c] = JJt(i,j);
	aa_la_inv(6, aminoInv);
	MatrixXd JJtinv (6,6);
	for(size_t i = 0, c = 0; i < 6; i++)
		for(size_t j = 0; j < 6; j++, c++)
			JJtinv(i,j) = aminoInv[c];
	MatrixXd Jinv = Jt * JJtinv;
	MatrixXd JinvJ = Jinv*J;
	MatrixXd I = MatrixXd::Identity(7,7);
	qdot = Jinv * xdot + (I - JinvJ) * qdot_nullspace * nullspace_gain;
}

/* ******************************************************************************************** */
void WorkspaceControl::update (const VectorXd& ui, const VectorXd& ft, 
		const VectorXd& qdot_secondary, double dt, VectorXd& qdot) {

	// Scale the ui input to get a workspace velocity 
	Eigen::VectorXd xdot_ui = ui;
	xdot_ui.topLeftCorner<3,1>() *= ui_translation_gain;
	xdot_ui.bottomLeftCorner<3,1>() *= ui_orientation_gain;
	if(debug) DISPLAY_VECTOR(xdot_ui);

	// Move the workspace references around from that ui input
	integrateWSVelocityInput(xdot_ui, dt);
	if(debug) DISPLAY_MATRIX(Tref);

	// Compute an xdot for complying with external forces if the f/t values are within thresholds
	Eigen::VectorXd xdot_comply;
	if(debug) DISPLAY_VECTOR(ft);
	xdot_comply = -ft * compliance_gain;
	if(debug) DISPLAY_VECTOR(xdot_comply);

	// Get an xdot out of the P-controller that's trying to drive us to the refernece position
	Eigen::VectorXd xdot_posref;
	refWSVelocity(xdot_posref);
	if(debug) DISPLAY_VECTOR(xdot_posref);

	// Combine the velocities from the workspace position goal, the ui, and the compliance
	Eigen::VectorXd xdot_apply = xdot_posref + xdot_ui + xdot_comply;
	if(debug) DISPLAY_VECTOR(xdot_apply);

	// Compute qdot with the dampened inverse Jacobian, using nullspace projection to achieve our 
	// secondary goal
	refJSVelocity(xdot_apply, qdot_secondary, qdot);
}

};	// end of namespace
