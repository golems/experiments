/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** \file WorkspaceControl.h
 *
 *  \author Jonathan Scholz
 *  \date 7/17/2013
 */

#ifndef WORKSPACETELEOP_H_
#define WORKSPACETELEOP_H_

#include <Eigen/Dense>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>

#include "KrangControl.h"

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
	void initialize(KrangControl *krang);

	// Update methods
	void updateRelativeTransforms();
	void updateXrefFromXdot(lwa_arm_t arm, Eigen::VectorXd &xdot);
	void updateXrefFromOther(lwa_arm_t arm, lwa_arm_t other);

	// getters and setters
	void setXref(lwa_arm_t arm, Eigen::Matrix4d &T);
	Eigen::Matrix4d getXref(lwa_arm_t arm);
	void setXcur(lwa_arm_t arm, Eigen::Matrix4d &T);
	Eigen::Matrix4d getXcur(lwa_arm_t arm);
	void setXrefFromOffset(lwa_arm_t arm, Eigen::Matrix4d &T); ///< set xref as offset from xcur

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
	Eigen::VectorXd xdotToQdot(lwa_arm_t arm, const Eigen::VectorXd &xdot, double nullGain = 0.01);

protected:
	// initialization helpers
	void initializeTransforms();

	// update helpers
	void setRelativeTransforms();
//	void setEffectorTransformFromSkel(lwa_arm_t arm, kinematics::BodyNode* eeNode);

	// Returns an xdot for the given arm's current reference position
	Eigen::VectorXd getXdotFromXref(lwa_arm_t arm);

	const std::vector<Eigen::Matrix4d>& getInitTrans() const {
		return initTrans;
	}

	void setInitTrans(const std::vector<Eigen::Matrix4d>& initTrans) {
		this->initTrans = initTrans;
	}

	// flag for whether to have left arm track the right one (TODO de-hackify)
	static const bool right_track_left_mode = 0;

	// Effector transforms
	Eigen::Matrix4d T_dummy;				///< helper identity transform TODO make const (initialization list?)
	std::vector<Eigen::Matrix4d> curTrans; 	///< current effector transforms
	std::vector<Eigen::Matrix4d> refTrans; 	///< reference transforms (ie goal)
	std::vector<Eigen::Matrix4d> relTrans; 	///< named effector transform in other effector frame
	std::vector<Eigen::Matrix4d> initTrans;	///< initial effector transform for position teleop (eg liberty)

private:
	// All important krang pointer
	KrangControl* _krang;
};

#endif /* WORKSPACETELEOP_H_ */
