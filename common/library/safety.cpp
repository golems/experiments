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

#include "safety.h"

namespace Krang {

/* ******************************************************************************************** */
bool checkCurrentLimits (const VectorXd& cur) {

	for(size_t i = 0; i < cur.size(); i++) {
		if(fabs(cur(i)) > CURRENT_WARN_LIMIT)
			printf("\t\t\tWARNING: Current at module %zu has passed %lf amps: %lf amps\n", i, 	
				CURRENT_WARN_LIMIT, cur(i));
		if(fabs(cur(i)) > CURRENT_KILL_LIMIT) {
			printf("\t\t\tStopping because current at module %zu has passed %lf amps: %lf amps\n", i,
				 CURRENT_KILL_LIMIT, cur(i));
			return true;
		}
	}
	return false;
}

/* ******************************************************************************************** */
void computeQdotAvoidLimits(dynamics::SkeletonDynamics* robot, const std::vector <int>& arm_ids, 
		const Eigen::VectorXd& q, Eigen::VectorXd& qdot_avoid) {
	double error, region;
	for(int i = 0; i < 7; i++) {
		// find our dof so we can get limits
		kinematics::Dof* dof = robot->getDof(arm_ids[i]);

		// store this so we aren't typing so much junk
		region = JOINTLIMIT_REGIONSIZE[i];

		// no avoidance if we arne't near a limit
		qdot_avoid[i] = 0.0;

		// if we're close to hitting the lower limit, move positive
		if (q[i] < dof->getMin()) qdot_avoid[i] = JOINTLIMIT_MAXVEL;
		else if (q[i] < dof->getMin() + region) {
			// figure out how close we are to the limit.
			error = q[i] - dof->getMin();

			// take the inverse to get the magnitude of our response
			// subtract 1/region so that our response starts at zero
			qdot_avoid[i] = (JOINTLIMIT_GAIN / error) - (JOINTLIMIT_GAIN / region);

			// clamp it to the maximum allowed avoidance strength
			qdot_avoid[i] = std::min(qdot_avoid[i], JOINTLIMIT_MAXVEL);
		}

		// if we're close to hitting the lower limit, move positive
		if (q[i] > dof->getMax()) qdot_avoid[i] = -JOINTLIMIT_MAXVEL;
		else if (q[i] > dof->getMax() - region) {
			// figure out how close we are to the limit.
			error = q[i] - dof->getMax();

			// take the inverse to get the magnitude of our response
			// subtract 1/region so that our response starts at zero
			qdot_avoid[i] = (JOINTLIMIT_GAIN / error) + (JOINTLIMIT_GAIN / region);

			// clamp it to the maximum allowed avoidance strength
			qdot_avoid[i] = std::max(qdot_avoid[i], -JOINTLIMIT_MAXVEL);
		}
	}
}
/* ******************************************************************************************** */
};
