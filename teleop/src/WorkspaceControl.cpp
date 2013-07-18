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

	curTrans.push_back(T_dummy);
	curTrans.push_back(T_dummy);

	refTrans.push_back(T_dummy);
	refTrans.push_back(T_dummy);

	relTrans.push_back(T_dummy);
	relTrans.push_back(T_dummy);

	// set initial poses
	curTrans[LEFT_ARM] = _krang->getEffectorPose(LEFT_ARM);
	curTrans[RIGHT_ARM] = _krang->getEffectorPose(RIGHT_ARM);

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
	////////////// VERSION 1: something dumb
//	setEffectorTransformFromSkel(arm);
//	Eigen::Matrix4d xdotM = eulerToTransform(xdot, math::XYZ);
//	refTrans[arm] = xdotM * curTrans[arm];

////////////// VERSION 2: apply to current pose
//	Eigen::Matrix4d curRot = curTrans[arm];
//	curRot.topRightCorner<3,1>().setZero();
//	refTrans[arm] = curTrans[arm] * curRot.inverse() * xdotM;

////////////// VERSION 2.5: apply to ref pose
	Eigen::Matrix4d refRot = refTrans[arm];
	refRot.topRightCorner<3,1>().setZero();
	Eigen::Matrix4d xdotM = eulerToTransform(xdot, math::XYZ);
	refTrans[arm] = refTrans[arm] * refRot.inverse() * xdotM;

	//TODO write a custom eulerToTransform
	//TODO also try not using transforms at all for reference
	//TODO or try doing feedforward in jointspace while we're just using euler for refs
}

Eigen::VectorXd WorkspaceControl::getXdotFromXref(lwa_arm_t arm) {
	curTrans[arm] = _krang->getEffectorPose(arm);

////////////// VERSION 1: something dumb
//	Eigen::Matrix4d xdotM = refTrans[arm] * curTrans[arm].inverse();
//	return transformToEuler(xdotM, math::XYZ);

////////////// VERSION 2: reverse of above (is actually a reference, but crappy Euler conversion)
	Eigen::Matrix4d curRot = curTrans[arm];
	curRot.topRightCorner<3,1>().setZero();
	Eigen::Matrix4d xdotM = curRot * curTrans[arm].inverse() * refTrans[arm];
	return transformToEuler(xdotM, math::XYZ);

////////////// VERSION 3: same as version 2, but with quaternions
//	Eigen::VectorXd goalPos = refTrans[arm].topRightCorner<3,1>();
//	Eigen::VectorXd eePos = curTrans[arm].topRightCorner<3,1>();
//	Eigen::VectorXd errPos = goalPos - eePos;
//
//	// compute rotation error using quaternions
//	Eigen::Quaternion<double> goalOri(refTrans[arm].topLeftCorner<3,3>());
//	Eigen::Quaternion<double> eeOri(curTrans[arm].topLeftCorner<3,3>());
//	Eigen::Quaternion<double> errOriQ = goalOri * eeOri.inverse();
//	Eigen::Matrix3d errOriM = errOriM = errOriQ.matrix();
//	Eigen::Vector3d errOri = math::matrixToEuler(errOriM, math::XYZ);
//
//	Eigen::VectorXd xdot(6);
//	xdot.setZero();
//	xdot << errPos, errOri;
//
//	return xdot;
}


/*
 * Sets xref of arm2 to track arm1
 *
 * NOTE:
 * everything after other arm is for ff velocity control
 */
void WorkspaceControl::updateXrefFromOther(lwa_arm_t arm, lwa_arm_t other,
		Eigen::VectorXd *qdotOther, Eigen::VectorXd *qOther, double dt) {

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
Eigen::VectorXd WorkspaceControl::xdotToQdot(lwa_arm_t arm, kinematics::BodyNode* eeNode, double xdotGain,
		double nullGain, Eigen::VectorXd *q, Eigen::VectorXd *xdot) {

	// obtain xdot from user or reference
	Eigen::VectorXd xd(6);
	if (xdot == NULL)
		xd = getXdotFromXref(arm);
	else
		xd = *xdot;

	// Get the Jacobian towards computing joint-space velocities
	Eigen::MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	Eigen::MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	Eigen::MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jsq = J * Jt;
	for (int i=0; i < Jsq.rows(); i++)
		Jsq(i,i) += 0.005;
	Eigen::MatrixXd Jinv = Jt * Jsq.inverse();

	// return Jinv * xdot; // simple version

	// Compute Joint Distance from middle of range
	Eigen::VectorXd qDist(7); qDist.setZero(7);
	if (q != NULL)
		qDist = q->cwiseAbs();

	Eigen::MatrixXd JinvJ = Jinv*J;
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7,7);

	Eigen::VectorXd qdot = Jinv * (xd * xdotGain);// + (I - JinvJ) * (qDist * nullGain);

	return qdot;
}

void WorkspaceControl::setRelativeTransforms() {

	// cache the relative effector transforms
	relTrans[LEFT_ARM] = curTrans[RIGHT_ARM].inverse() * curTrans[LEFT_ARM]; ///< left effector transform in right effector frame
}
