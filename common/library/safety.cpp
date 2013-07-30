/**
 * @file safety.h
 * @author Can Erdogan, Saul Reynolds-Haertle
 * @date July 27, 2013
 * @brief This file contains the functions that exert safety checks on inputs to the motors.
 */

#include "safety.h"
#include "display.hpp"

namespace Krang {

/* ******************************************************************************************** */
bool checkCurrentLimits (const VectorXd& cur) {

	for(size_t i = 0; i < cur.size(); i++) {
		if(fabs(cur(i)) > CURRENT_WARN_LIMIT) {
			if(doing_curses) {
			} else {
				printf("\t\t\tWARNING: Current at module %zu has passed %lf amps: %lf amps\n", i, 	
				       CURRENT_WARN_LIMIT, cur(i));
			}
		}
		if(fabs(cur(i)) > CURRENT_KILL_LIMIT) {
			if(doing_curses) {
			} else {
				printf("\t\t\tStopping because current at module %zu has passed %lf amps: %lf amps\n", i,
				       CURRENT_KILL_LIMIT, cur(i));
			}
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
