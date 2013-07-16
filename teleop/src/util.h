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

/************************ Helpers **************************/

/*
 * Converts a 4x4 homogeneous transform to a 6D euler.
 * Conversion convention corresponds to Grip's ZYX
 */
VectorXd transformToEuler(const MatrixXd &T, math::RotationOrder _order) { // math::XYZ
	// extract translation
	Vector3d posV = T.topRightCorner<3,1>();

	// convert rotmat to euler
	Matrix3d rotM = T.topLeftCorner<3,3>();
	Vector3d rotV = math::matrixToEuler(rotM, _order);  // math::ZYX for spacenav!?

	// pack into a 6D config vector
	VectorXd V(6);
	V << posV, rotV;
	return V;
}

MatrixXd eulerToTransform(const VectorXd &V, math::RotationOrder _order) { // math::XYZ
	// extract translation
	Vector3d posV; posV << V[0], V[1], V[2];

	// extract rotation
	Vector3d rotV; rotV << V[3], V[4], V[5];

	// convert rotmat to euler
	Matrix3d rotM = math::eulerToMatrix(rotV, _order);  // math::ZYX for spacenav!?

	// pack into a 4x4 matrix
	MatrixXd T(4,4);
	T.topLeftCorner<3,3>() = rotM;
	T.topRightCorner<3,1>() = posV;
	T(3,3) = 1.0;

	return T;
}



#endif /* UTIL_H_ */
