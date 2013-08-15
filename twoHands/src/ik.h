/**
 * @file ik.h
 * @author Can Erdogan
 * @date May 03, 2013
 * @brief Handles inverse-kinematics operation for a 7-dof arm with Krang's dimensions.
 */

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dynamics/SkeletonDynamics.h>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include <math/UtilsRotation.h>

using namespace dynamics;
using namespace simulation;
using namespace Eigen;

/// Makes the small 1e-17 values in a matrix zero for printing
MatrixXd fix (const MatrixXd& mat);

#define RAD2DEG(x) ((x) / M_PI * 180.0)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)
#define pv(a) std::cout << #a << ": " << fix((a).transpose()) << "\n" << std::endl
#define pc(a) std::cout << #a << ": " << (a) << "\n" << std::endl
#define pm(a) std::cout << #a << ":\n " << fix((a).matrix()) << "\n" << std::endl
#define pmr(a) std::cout << #a << ":\n " << fix((a)) << "\n" << std::endl

// The link lengths
static const double theta1Offset = 0.1838;		

static const double L1 = .5074;				///> Distance between wheel axes and the waist module
static const double L2 = .1098;				///> Distance between the waist module and the spine
static const double L3 = .6694;				///> Dist. btw torso frame's origin and the bases of the arms
static const double L4 = .2856;				///> Arm link length between modules 0 and 2
static const double L5 = .3278;				///> Arm link length between modules 2 and 4
static const double L6 = .2763;				///> Arm link length between modules 4 and 6
static const double L7 = .2220;				///> Arm link length between modules 6 and end-effector
static const double L8_Schunk = 0.1750;	///> The length from f/t to middle of schunk gripper metals

/// The A matrix is the 4x4 transformation that represents the proximal elbow frame in the
/// distal shoulder frame. In Krang, this represents the 6th frame in the 5th frame.
static const Matrix4d A = (MatrixXd(4,4) << 1,0,0,0,0,0,1,0,0,-1,0,-L5,0,0,0,1).finished();

/// The B matrix is the 4x4 transformation that represents the proximal wrist frame in the
/// distal elbow frame. In Krang, this represents the 7th frame in the 'rotated' 6th frame.
static const Matrix4d B = (MatrixXd(4,4) << 1,0,0,0,0,0,-1,L6,0,1,0,0,0,0,0,1).finished();

/// Returns the 6d vector from 4d matrix
inline Matrix<double, 6, 1> matToVec (const Matrix4d& M) {
	Matrix3d rot = M.topLeftCorner<3,3>();
	Vector3d rotv = dart_math::matrixToEuler(rot, dart_math::XYZ) / M_PI * 180.0;
	Matrix<double, 6, 1> res;
	res << M.topRightCorner<3,1>(), rotv;	
	return res;
}

/// Returns the angles of a rotation matrix with respect to three axes n1, n2 and n2 such that
/// n1 \perpto n2 and n2 \perpto n3. 
void anglesFromRotationMatrix(double &theta1, double &theta2, double &theta3, const Vector3d &n1, 
		const Vector3d &n2, const Vector3d &n3, const Matrix3d &A);

/// Computes a transformation given the DH parameters
Matrix4d dh (double a, double alpha, double d, double theta);

/// Implements Sections 3.2.1 and 3.2.2
void getElbowPosAngle (const Vector3d& relGoalLoc, double Lsw, double phi, Vector3d& elbow, 
		double& theta4);

/// Returns the transformation between the proximal and distal shoulder frames (3.2.3)
void getT1 (const Transform<double, 3, Affine>& relGoal, const Transform<double, 3, Affine>& Ty,
		const Vector3d& elbow, Transform<double, 3, Affine>& T1);

/// Performs I.K. for a single arm and returns true if successful. If not successful, sets the arm
/// to the initial condition.
bool singleArmIK (simulation::World* world, SkeletonDynamics* robot, const Matrix4d& Twee, 
		bool rightArm);

/// Computes the inverse-kinematics for the wrist-elbow-shoulder case. SingleArmIK calls this.
bool ik (const Transform<double, 3, Affine>& goal, double phi, Matrix <double, 7, 1>& theta);

/// Given the goal configuration in the world frame, returns the goal location of the wrist
/// in the shoulder frame.
void getWristInShoulder (SkeletonDynamics* robot, const Matrix4d& Twee, bool rightArm, 
		Matrix4d& TswM);

/// Given the goal configuration in the bracket frame, returns the goal location of the wrist
/// in the shoulder frame.
void getRelativeGoal(const Transform<double, 3, Affine>& goal, Transform<double, 3, Affine>& rel);

