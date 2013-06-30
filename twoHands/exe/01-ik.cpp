/**
 * @file 01-ik.cpp
 * @author Can Erdogan
 * @date May 01, 2013
 * @brief The goal of this executable is to show the successful execution of inverse kinematics for
 * a 7-DOF arm (specifically Krang's dimensions). For a given 6-DOF XYZRPY task-space goal, returns
 * 12 values around the elbow circle if the goal is reachable. If not, returns an error message.
 */

#include "ik.h"
#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace Eigen;
using namespace std;

/* ********************************************************************************************** */
/// Given the goal configuration in the bracket frame, returns the goal location of the wrist
/// in the shoulder frame.
void getRelativeGoal(const Transform<double, 3, Affine>& goal, Transform<double, 3, Affine>& rel) {
	
	// First of all, the rotation stays the same
	rel.linear() = goal.linear();

	// Next, get the wrist location by going backwards from the end-effector towards the wrist
	Vector3d posGoal = goal.translation() + rel.linear() * Vector3d(0.0, 0.0, L7);
	rel.translation() = posGoal;

	// Lastly, transform the matrix to the shoulder frame
	rel.translation() += Vector3d(0.0, 0.0, L4);
}

/* ********************************************************************************************** */
int main (int argc, char* argv[]) {

	// Set the goal transformation for the end-effector
	Transform<double, 3, Affine> goal; // (AngleAxis <double> (-M_PI/2.0, Vector3d::UnitY()));
	goal.setIdentity();
	goal.translation() = Vector3d(0.02, 0.03, -0.75);

	cout << "Goal position and orientation: \n" << goal.matrix() << "\n" << endl;

	// Compute the relative goal
	Transform<double, 3, Affine> relGoal;
	getRelativeGoal(goal, relGoal);
	pm(relGoal);

	// Set the plane angle and get the arm angles
	Matrix <double, 7, 1> theta;	
	const double phi = 0.0;
	ik(relGoal, phi, theta);

	// Print the results
	cout << "Output angles: " << theta.transpose() << "\n" << endl;

	// Check the elbow transformation
	Matrix4d fkResult = dh(0.0, -M_PI_2, -L4, theta(0)) * 
                        dh(0.0,  M_PI_2, 0.0, theta(1)) *
                        dh(0.0, -M_PI_2, -L5, theta(2)) *
												dh(0.0,  M_PI_2, 0.0, theta(3)) * 
												dh(0.0, -M_PI_2, -L6, theta(4)) * 
												dh(0.0,  M_PI_2, 0.0, theta(5)) * 
												dh(0.0, 0.0, -L7, theta(6));

	cout << "Result of forward kinematics: \n" << fix(fkResult) << endl;
}
