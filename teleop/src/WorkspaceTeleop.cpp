/*
 * WorkspaceTeleop.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#include "WorkspaceTeleop.h"
#include <math/UtilsRotation.h>

#include "util.h" //TODO: rename, move, or do something good

WorkspaceTeleop::WorkspaceTeleop(kinematics::BodyNode* eeNodeL, kinematics::BodyNode* eeNodeR) {
	eeNodes[LEFT_ARM] = eeNodeL;
	eeNodes[RIGHT_ARM] = eeNodeR;
	initializeTransforms();
	setEffectorTransformFromSkel(LEFT_ARM);
	setEffectorTransformFromSkel(RIGHT_ARM);
}

WorkspaceTeleop::~WorkspaceTeleop() {
	// TODO Auto-generated destructor stub
}

/*
 * Initializes all transform containers to the identity.
 * For now, we're simply assuming 2 possible arms
 */
void WorkspaceTeleop::initializeTransforms() {

	T_dummy.setIdentity();

	curTrans.push_back(T_dummy);
	curTrans.push_back(T_dummy);

	refTrans.push_back(T_dummy);
	refTrans.push_back(T_dummy);

	relTrans.push_back(T_dummy);
	relTrans.push_back(T_dummy);
}

void WorkspaceTeleop::setEffectorTransformFromSkel(lwa_arm_t arm) {

	assert(eeNodes[arm] != NULL);
	curTrans[arm] = eeNodes[arm]->getWorldTransform();
}

void WorkspaceTeleop::setEffectorTransform(lwa_arm_t arm, Eigen::Matrix4d T) {

	curTrans[arm] = T;
}

void WorkspaceTeleop::setXrefFromTransform(lwa_arm_t arm, Eigen::Matrix4d& T) {

	refTrans[arm] = T;
}

/*
 * Updates the reference transform for the target arm according to the provided
 * 6D config vector.  This vector is interpreted as a desired workspace velocity
 * for the goal reference, with orientation represented as EulerXYZ.
 *
 * Note: xdot should be appropriately scaled before passing in.  Internally it
 * is treated as a relative transform applied to the current effector pose
 */
void WorkspaceTeleop::updateXrefFromXdot(lwa_arm_t arm, Eigen::VectorXd& xdot) {

	Eigen::Matrix4d xdotM = eulerToTransform(xdot, math::XYZ);
	refTrans[arm] = xdotM * curTrans[arm];
}

Eigen::VectorXd WorkspaceTeleop::getXdotFromXref(lwa_arm_t arm) {

	// get transform from effector to reference in global frame
	Eigen::Matrix4d xdotM = refTrans[arm] * curTrans[arm].inverse();

	// return as a config vector
	return transformToEuler(xdotM, math::XYZ);
}

/*
 * Main work-horse function: converts workspace velocities to jointspace
 * using the arm jacobians, as obtained from Dart.
 *
 * This implementation performs a damped pseudoinverse of J, and does the
 * null-space projection thing to bias our solution towards
 * joint values in the middle of each joint's range of motion
 */
Eigen::VectorXd WorkspaceTeleop::xdotToQdot(lwa_arm_t arm, const Eigen::VectorXd& xdot,
		double xdotGain, double nullGain, Eigen::VectorXd *q) {


	// Get the Jacobian towards computing joint-space velocities
	Eigen::MatrixXd Jlin = eeNodes[arm]->getJacobianLinear().topRightCorner<3,7>();
	Eigen::MatrixXd Jang = eeNodes[arm]->getJacobianAngular().topRightCorner<3,7>();
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
	return Jinv * (xdot * xdotGain) + (I - JinvJ) * (qDist * nullGain);
}

void WorkspaceTeleop::setRelativeTransforms() {

	// cache the relative effector transforms
	relTrans[LEFT_ARM] = curTrans[RIGHT_ARM].inverse() * curTrans[LEFT_ARM]; ///< left effector transform in right effector frame
}