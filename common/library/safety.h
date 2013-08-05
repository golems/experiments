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
 * @file safety.h
 * @author Can Erdogan, Saul Reynolds-Haertle
 * @date July 27, 2013
 * @brief This file contains the functions that exert safety checks on inputs to the motors.
 */

#include "util.h"

namespace Krang {

	/// The boundary at the joint limits where the joint limit avoidance starts advicing a nonzero	
	/// velocity in the opposite direction
	const Eigen::VectorXd JOINTLIMIT_REGIONSIZE = 
		(VectorXd(7) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished();

	/// The maximum velocity at which we oppose joint limits
	const double JOINTLIMIT_MAXVEL = 1.0; 

	/// The steepness of the rewaction curve (rigidity of 'spring' that pushes away from the limit).
	const double JOINTLIMIT_GAIN = 0.01; 

	/// The warning and kill limits for current checks
	const double CURRENT_WARN_LIMIT = 10.0; 
	const double CURRENT_KILL_LIMIT = 12.0;
	
	/// Monitors the current values and either prints warnings or returns a boolean which indicates the program
	/// should be stopped immediately
	bool checkCurrentLimits (const VectorXd& cur);

	/// Returns a jointspace velocity to move with to oppose the joint limit in a smooth way
	/// Basically, an inverse distance to boundary function which returns higher velocity as the 
	/// joint limit is approached
	void computeQdotAvoidLimits(dynamics::SkeletonDynamics* robot, const std::vector <int>& arm_ids, 
			const Eigen::VectorXd& q, Eigen::VectorXd& qdot_avoid);

};	 // end of namespace
