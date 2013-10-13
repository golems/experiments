/**
 * @file helpers.h
 * @author Can Erdogan
 * @date June 12, 2013
 * @brief This file contains some helper functions such as reading force/torque data if available.
 */

#pragma once

#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <syslog.h>
#include <fcntl.h>
#include <argp.h>

#include <Eigen/Dense>

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <imud.h>

#include "initModules.h"
#include "motion.h"
#include <iostream>

#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

using namespace Eigen;
using namespace std; 

/* ******************************************************************************************** */
#define pv(x) std::cout << #x << ": " << (x).transpose() << std::endl;

#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
	llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);

#define darm (cout << "dq: "<<llwa.vel[0] << ", " <<llwa.vel[1] << ", " << llwa.vel[2] << ", " << \
	llwa.vel[3] << ", " << llwa.vel[4] << ", " << llwa.vel[5] << ", " << llwa.vel[6] << endl);

#define eig7(x) (Vector7d() << (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5], (x)[6]).finished()

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION

/* ******************************************************************************************** */
typedef Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain joint values
typedef Matrix<double, 6, 6> Matrix6d;			///< A typedef for convenience to contain wrenches

/// A useful struct to contain a single arm information
struct Arm {
	somatic_motor_t lwa;					///< the motor control for the arm
	ach_channel_t ft_chan;				///< the channel for the f/t sensor
 	Vector6d offset;							///< the offset in f/t readings from the truth
};

/* ******************************************************************************************** */
static const double eeMass = 1.6 + 0.169 + 0.000;			///< The mass of the end-effector
static simulation::World *world = NULL;						///< The dart environment loaded for kinematics
static std::vector <int> left_arm_ids;				///< The index vector to set config of arms
static std::vector <int> right_arm_ids;				///< The index vector to set config of arms

/* ******************************************************************************************** */
/// Set the vector from the sensor origin to the gripper center of mass (m)
static const Vector3d s2com (0.0, 0.0, 0.09); // 0.0683 schunk itself, 0.026 length of ext + 2nd

/// Initializes the daemon, joystick/ft channels, left arm and computes the initial offset for ft
void init (somatic_d_t& daemon_cx, ach_channel_t& js_chan, ach_channel_t& imuChan, 
		ach_channel_t& waistChan, Arm* left = NULL, Arm* right = NULL);

/* ******************************************************************************************** */
// Functions to process data as in computing external wrenches or what to do with them

/// Given a wrench, computes the joint space velocities so that wrench is minimized 

/// Computes the initial offset from the given first raw value
void computeOffset (double imu, double waist, const somatic_motor_t& llwa, const Vector6d& raw, 
		dynamics::SkeletonDynamics& robot, Vector6d& offset, bool left);

/// Computes the external force and torque from the values assuming that the input is already
/// corrected for the effect of gravity. That is the arm readings should reflect the weight
/// of the end-effector when there are no external inputs.
void computeExternal (double imu, double waist, const somatic_motor_t& llwa, const Vector6d& input, 
		dynamics::SkeletonDynamics& robot, Vector6d& external, bool left);

/* ******************************************************************************************** */
// Functions to retrieve data

/// Returns the f/t data if available at that instance
bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data);

/// Reads imu channel 
void getImu(double* imu, ach_channel_t& imuChan);

/// Reads waist channel 
bool getWaist(double* waist, ach_channel_t& waistChan);

/* ******************************************************************************************** */
/// Given a wrench, computes the joint space velocities so that wrench is minimized 
static void wrenchToJointVels (const Vector6d& wrench, Vector7d& dq, bool left) {

	// Get the Jacobian towards computing joint-space velocities
	const char* nodeName = left ? "lGripper" : "rGripper";
	kinematics::BodyNode* eeNode = world->getSkeleton(0)->getNode(nodeName);
	MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	vector <int> dofs;   
 	for(size_t i = 0; i < 24; i++) dofs.push_back(i);

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jinv = Jt * (J * Jt).inverse();

	// Get the joint-space velocities by multiplying inverse Jacobian with the opposite wrench.
	dq = Jinv * wrench / 300.0;

	// Threshold the velocities
	double limit = 0.2;
	for(size_t i = 0; i < 7; i++) {
		if(dq(i) > limit) dq(i) = limit;
		else if(dq(i) < -limit) dq(i) = -limit;
	}
}


