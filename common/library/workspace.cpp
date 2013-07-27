/**
 * @file workspace.cpp
 * @author Can Erdogan, Saul Reynolds-Haertle
 * @date July 27, 2013
 * @brief This header file for workspace control of 7-dof arms.
 */

#include "workspace.h"

namespace Krang {

/* ******************************************************************************************** */
WorkspaceControl::WorkspaceControl (dynamics::SkeletonDynamics* robot, Side side, 
		double _K_posRef_p, double _nullspace_gain, double _damping_gain) {

	// Determine the end-effector and the arm indices based on the input side
	endEffector = robot->getNode((side == LEFT) ? "lGripper" : "rGripper");
	arm_ids = (side == LEFT) ? &left_arm_ids : &right_arm_ids;
	
	// Set the gains
	K_posRef_p = _K_posRef_p, nullspace_gain = _nullspace_gain, damping_gain = _damping_gain;
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
	MatrixXd Jinv = Jt * JJt.inverse();
	MatrixXd JinvJ = Jinv*J;
	MatrixXd I = MatrixXd::Identity(7,7);
	qdot = Jinv * xdot + (I - JinvJ) * qdot_nullspace * nullspace_gain;
}

};
