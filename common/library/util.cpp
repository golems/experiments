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
 * @file common.cpp
 * @author Can Erdogan
 * @date July 25, 2013
 * @brief The main source file for common utilities
 */

#include <vector>
#include <Eigen/Dense>

#include "util.h"

using namespace std;

namespace Krang {

/* ******************************************************************************************** */
// Setup the indices for the motor groups

int left_arm_ids_a [7] = {11, 13, 15, 17, 19, 21, 23}; 
int right_arm_ids_a [7] = {12, 14, 16, 18, 20, 22, 24}; 
int imuWaist_ids_a [2] = {5, 8};
vector <int> left_arm_ids (left_arm_ids_a, left_arm_ids_a + 7);						
vector <int> right_arm_ids (right_arm_ids_a, right_arm_ids_a + 7);	
vector <int> imuWaist_ids (imuWaist_ids_a, imuWaist_ids_a + 2);		


/* ******************************************************************************************** */
// set up the indices for dart's root dofs

int dart_root_dof_ids_a[] =  {0,1,2,5,4,3};
std::vector<int> dart_root_dof_ids(dart_root_dof_ids_a, dart_root_dof_ids_a + 6);

/* ******************************************************************************************** */
Eigen::VectorXd transformToEuler(const Eigen::MatrixXd &T, math::RotationOrder _order) {

	// Extract the translation
	Eigen::Vector3d posV = T.topRightCorner<3,1>();

	// Convert the rotation matrix into the RPY representation
	Eigen::Matrix3d rotM = T.topLeftCorner<3,3>();
	Eigen::Vector3d rotV = math::matrixToEuler(rotM, _order);

	// Pack into a 6D config vector
	Eigen::VectorXd V(6);
	V << posV, rotV;
	return V;
}

/* ******************************************************************************************** */
Eigen::MatrixXd eulerToTransform(const Eigen::VectorXd &V, math::RotationOrder _order) {

	// Extract the translation
	Eigen::Vector3d posV; posV << V[0], V[1], V[2];

	// Extract the rotation and make a matrix out of it
	Eigen::Vector3d rotV; rotV << V[3], V[4], V[5];
	Eigen::Matrix3d rotM = math::eulerToMatrix(rotV, _order);

	// Pack the values in a 4x4 matrix
	Eigen::MatrixXd T(4,4);
	T.topLeftCorner<3,3>() = rotM;
	T.topRightCorner<3,1>() = posV;
	T.row(3) << 0,0,0,1;
	return T;
}

/* ******************************************************************************************** */
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < 1e-5) mat2(i,j) = 0.0;
	return mat2;
}

};
