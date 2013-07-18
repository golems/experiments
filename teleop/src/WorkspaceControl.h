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

class WorkspaceControl {
public:
	WorkspaceControl();
	virtual ~WorkspaceControl();

	// initialization
	void initialize(kinematics::BodyNode* eeNodeL = NULL, kinematics::BodyNode* eeNodeR = NULL);

	// Update methods
	void updateRelativeTransforms();
	void updateXrefFromXdot(lwa_arm_t arm, Eigen::VectorXd &xdot);
	void updateXrefFromOther(lwa_arm_t arm, lwa_arm_t other);

	// getters and setters
	void setXref(lwa_arm_t arm, Eigen::Matrix4d &T);
	Eigen::Matrix4d getXref(lwa_arm_t arm);
	void setXcur(lwa_arm_t arm, Eigen::Matrix4d &T);
	Eigen::Matrix4d getXcur(lwa_arm_t arm);


	/**
	 * Workhorse function: maps given xdot into joint space.
	 *
	 * @param arm: An arm to control (left or right)
	 * @param xdotGain: Jointspace velocity gain
	 * @param nullGain: A nullspace projection gain (how strongly to bias towards zero)
	 * @param q: The current joint angles (TODO: add way to bias towards arbitrary)
	 * @param xdot: A desired xdot.  If Null, it is computed from the arm's current reference
	 * @return: the jointspace velocities qdot
	 */
	Eigen::VectorXd xdotToQdot(lwa_arm_t arm, double xdotGain, double nullGain = 0.01,
			Eigen::VectorXd *q = NULL, Eigen::VectorXd *xdot = NULL);

protected:
	// initialization helpers
	void initializeTransforms();

	// update helpers
	void setRelativeTransforms();
	void setEffectorTransformFromSkel(lwa_arm_t arm);

	// Returns an xdot for the given arm's current reference position
	Eigen::VectorXd getXdotFromXref(lwa_arm_t arm);

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
