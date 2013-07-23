/*

 * WorkspaceTeleop.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#include "WorkspaceControl.h"
#include <math/UtilsRotation.h>

#include "util.h" //TODO: rename, move, or do something good


WorkspaceControl::WorkspaceControl() {

}

WorkspaceControl::~WorkspaceControl() {
	// TODO Auto-generated destructor stub
}

void WorkspaceControl::initialize(KrangControl *krang) {
	this->_krang = krang;
	initializeTransforms();
}

/*
 * Initializes all transform containers to the identity.
 * For now, we're simply assuming 2 possible arms
 */
void WorkspaceControl::initializeTransforms() {

	T_dummy.setIdentity();

	curTrans.resize(0);
	curTrans.push_back(T_dummy);
	curTrans.push_back(T_dummy);

	refTrans.resize(0);
	refTrans.push_back(T_dummy);
	refTrans.push_back(T_dummy);

	relTrans.resize(0);
	relTrans.push_back(T_dummy);
	relTrans.push_back(T_dummy);

	initTrans.resize(0);
	initTrans.push_back(T_dummy);
	initTrans.push_back(T_dummy);

	// set initial poses
	initTrans[LEFT_ARM] = _krang->getEffectorPose(LEFT_ARM);
	initTrans[RIGHT_ARM] = _krang->getEffectorPose(RIGHT_ARM);
	curTrans[LEFT_ARM] = initTrans[LEFT_ARM];
	curTrans[RIGHT_ARM] = initTrans[RIGHT_ARM];

	// set references to current poses
	refTrans[LEFT_ARM] = curTrans[LEFT_ARM];
	refTrans[RIGHT_ARM] = curTrans[RIGHT_ARM];
}

void WorkspaceControl::updateRelativeTransforms() {
	relTrans[LEFT_ARM] = curTrans[RIGHT_ARM].inverse() * curTrans[LEFT_ARM];
	relTrans[RIGHT_ARM] = curTrans[LEFT_ARM].inverse() * curTrans[RIGHT_ARM];
}

void WorkspaceControl::setXref(lwa_arm_t arm, Eigen::Matrix4d& T) {

	refTrans[arm] = T;
}

Eigen::Matrix4d WorkspaceControl::getXref(lwa_arm_t arm) {
	return refTrans[arm];
}

void WorkspaceControl::setXcur(lwa_arm_t arm, Eigen::Matrix4d& T) {

	curTrans[arm] = T;
}

Eigen::Matrix4d WorkspaceControl::getXcur(lwa_arm_t arm) {
	return curTrans[arm];
}

/*
 * Updates the reference transform for the target arm according to the provided
 * 6D config vector.  This vector is interpreted as a desired workspace velocity
 * for the goal reference, with orientation represented as EulerXYZ.
 *
 * Note: xdot should be appropriately scaled before passing in.  Internally it
 * is treated as a relative transform applied to the current effector pose
 */
void WorkspaceControl::updateXrefFromXdot(lwa_arm_t arm, Eigen::VectorXd& xdot) {
	// extract the rotational component as a transform, so we can express xdot in the global frame
	Eigen::Matrix4d refRot = refTrans[arm];
	refRot.topRightCorner<3,1>().setZero();
	Eigen::Matrix4d xdotM = eulerToTransform(xdot, math::XYZ);

	// apply xdot, expressed in the global frame, to the reference transform
	refTrans[arm] = refTrans[arm] * refRot.inverse() * xdotM * refRot;
}

/*
 * Returns a workspace velocity xdot to move the arm towards its reference
 * pose from its current pose.
 */
Eigen::VectorXd WorkspaceControl::getXdotFromXref(lwa_arm_t arm) {

	// update current transform of the arm we're moving
	curTrans[arm] = _krang->getEffectorPose(arm);

	//
	Eigen::Matrix4d curRot = curTrans[arm];
	curRot.topRightCorner<3,1>().setZero();
	Eigen::Matrix4d xdotM = curRot * curTrans[arm].inverse() * refTrans[arm] * curRot.inverse();
	return transformToEuler(xdotM, math::XYZ);
}


/*
 * Sets xref of arm2 to track arm1
 *
 * NOTE:
 * everything after other arm is for ff velocity control
 */
void WorkspaceControl::updateXrefFromOther(lwa_arm_t arm, lwa_arm_t other) {

	// set arm reference based on cached relative transform
	refTrans[arm] = refTrans[other] * relTrans[arm];
}

/*
 * Main work-horse function: converts workspace velocities to jointspace
 * using the arm jacobians, as obtained from Dart.
 *
 * This implementation performs a damped pseudoinverse of J, and does the
 * null-space projection thing to bias our solution towards
 * joint values in the middle of each joint's range of motion
 */
Eigen::VectorXd WorkspaceControl::xdotToQdot(lwa_arm_t arm, const Eigen::VectorXd &xdot, double nullGain) {

	Eigen::MatrixXd J = _krang->getEffectorJacobian(arm);

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jsq = J * Jt;
	for (int i=0; i < Jsq.rows(); i++)
		Jsq(i,i) += 0.005;
	Eigen::MatrixXd Jinv = Jt * Jsq.inverse();

	// return Jinv * xdot; // simple version

	// Compute Joint Distance from middle of range
	Eigen::VectorXd qDist(7); qDist.setZero(7);
	Eigen::VectorXd q = _krang->getArmConfig(arm);
	qDist = q.cwiseAbs2();

	Eigen::MatrixXd JinvJ = Jinv*J;
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7,7);

	Eigen::VectorXd qdot = Jinv * xdot  + (I - JinvJ) * (qDist * nullGain);

	return qdot;
}

void WorkspaceControl::setRelativeTransforms() {

	// cache the relative effector transforms
	relTrans[LEFT_ARM] = curTrans[RIGHT_ARM].inverse() * curTrans[LEFT_ARM]; ///< left effector transform in right effector frame
}

/*
 * Sets xref using T as a global-frame offset from xcur
 */
void WorkspaceControl::setXrefFromOffset(lwa_arm_t arm, Eigen::Matrix4d& T) {
	Eigen::Matrix4d initRot = initTrans[arm];
	initRot.topRightCorner<3,1>().setZero();

	//refTrans[arm] = curTrans[arm].inverse() * T * curTrans[arm];
	refTrans[arm] = initTrans[arm] * initRot.inverse() * T * initRot;
	//refTrans[arm] = initTrans[arm].inverse() * T * initTrans[arm];
}
