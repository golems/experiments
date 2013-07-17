/*
 * util.h
 *
 *  Created on: Jul 15, 2013
 *      Author: jscholz
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <Eigen/Dense>
#include <math/UtilsRotation.h>
#include <iomanip>
#include <vector>

/************************ Helpers **************************/
#define DISPLAY_VECTOR(VEC) std::cout << std::setw(24) << std::left << #VEC; for(int i = 0; i < VEC.size(); i++) std::cout << std::setw(12) << VEC[i]; std::cout << std::endl;

static const int vv[] =  {0,1,2,5,4,3};
static const std::vector<int> dartRootDofOrdering(&vv[0], &vv[0]+6);

/*
 * Converts a 4x4 homogeneous transform to a 6D euler.
 * Conversion convention corresponds to Grip's ZYX
 */
Eigen::VectorXd transformToEuler(const Eigen::MatrixXd &T, math::RotationOrder _order);
Eigen::MatrixXd eulerToTransform(const Eigen::VectorXd &V, math::RotationOrder _order);


#endif /* UTIL_H_ */
