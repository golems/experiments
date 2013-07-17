/*
 * WorkspaceTeleop.h
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#ifndef WORKSPACETELEOP_H_
#define WORKSPACETELEOP_H_

#include <Eigen/Dense>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>

//TODO: should dump this in a namespace
typedef enum arm {
	LEFT_ARM = 0,
	RIGHT_ARM
} lwa_arm_t;

/*
 *
 * Assumes access to a dart skeleton for each effector which can be
 * used to obtain the effector's current pose and jacobian.
 */

class WorkspaceTeleop {
public:
	WorkspaceTeleop(kinematics::BodyNode* eeNodeL = NULL, kinematics::BodyNode* eeNodeR = NULL);
	virtual ~WorkspaceTeleop();

	// Update methods
	void setEffectorTransformFromSkel(lwa_arm_t arm);
	void setEffectorTransform(lwa_arm_t arm, Eigen::Matrix4d T);
	void setXrefFromTransform(lwa_arm_t arm, Eigen::Matrix4d &T);
	void updateXrefFromXdot(lwa_arm_t arm, Eigen::VectorXd &xdot);

	// Jacobian workspace control methods
	Eigen::VectorXd getXdotFromXref(lwa_arm_t arm);

	///< Workhorse function: maps given xdot into joint space
	Eigen::VectorXd xdotToQdot(lwa_arm_t arm, const Eigen::VectorXd &xdot,
			double xdotGain, double nullGain, Eigen::VectorXd *q = NULL);

protected:
	// initialization helpers
	void initializeTransforms();

	// update helpers
	void setRelativeTransforms();

	// flag for whether to have left arm track the right one (TODO de-hackify)
	static const bool right_track_left_mode = 0;

	// Effector transforms
	Eigen::Matrix4d T_dummy;				///< helper identity transform TODO make const (initialization list?)
	std::vector<Eigen::Matrix4d> curTrans; 	///< current effector transforms
	std::vector<Eigen::Matrix4d> refTrans; 	///< reference transforms (ie goal)
	std::vector<Eigen::Matrix4d> relTrans; 	///< named effector transform in other effector frame

	// Pointers to frequently used dart data structures
	std::vector<kinematics::BodyNode*> eeNodes;
};

#endif /* WORKSPACETELEOP_H_ */
