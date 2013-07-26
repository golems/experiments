/*
 * util.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#include "util.h"
#include <iostream>
/*
 * Converts a 4x4 homogeneous transform to a 6D euler.
 * Conversion convention corresponds to Grip's ZYX
 */
Eigen::VectorXd transformToEuler(const Eigen::MatrixXd &T, math::RotationOrder _order) {
	// extract translation
	Eigen::Vector3d posV = T.topRightCorner<3,1>();

	// convert rotmat to euler
	Eigen::Matrix3d rotM = T.topLeftCorner<3,3>();
	Eigen::Vector3d rotV = math::matrixToEuler(rotM, _order);

	// pack into a 6D config vector
	Eigen::VectorXd V(6);
	V << posV, rotV;
	return V;
}

Eigen::MatrixXd eulerToTransform(const Eigen::VectorXd &V, math::RotationOrder _order) {
	// extract translation
	Eigen::Vector3d posV; posV << V[0], V[1], V[2];

	// extract rotation
	Eigen::Vector3d rotV; rotV << V[3], V[4], V[5];

	// convert rotmat to euler
	Eigen::Matrix3d rotM = math::eulerToMatrix(rotV, _order);

	// pack into a 4x4 matrix
	Eigen::MatrixXd T(4,4);
	T.topLeftCorner<3,3>() = rotM;
	T.topRightCorner<3,1>() = posV;
	T.row(3) << 0,0,0,1;

	return T;
}



