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

/* ******************************************************************************************** */
// Setup the indices for the motor groups

int left_arm_ids_a [7] = {11, 13, 15, 17, 19, 21, 23}; 
int right_arm_ids_a [7] = {12, 14, 16, 18, 20, 22, 24}; 
int imuWaist_ids_a [2] = {5, 8};
vector <int> left_arm_ids (left_arm_ids_a, left_arm_ids_a + 7);						
vector <int> right_arm_ids (right_arm_ids_a, right_arm_ids_a + 7);	
vector <int> imuWaist_ids (imuWaist_ids_a, imuWaist_ids_a + 2);		

/* ******************************************************************************************** */
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < 1e-5) mat2(i,j) = 0.0;
	return mat2;
}



