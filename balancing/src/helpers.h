/**
 * @file helpers.h
 * @author Munzir
 * @date July 8th, 2013
 * @brief This file comtains some helper functions used for balancing	
 */

#pragma once

#include <Eigen/Dense>

#include <dirent.h>
#include <iostream>

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <filter.h>
#include <imud.h>
#include <pciod.h>

#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <initModules.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>

using namespace Eigen;
using namespace dynamics;

bool myDebug;
extern bool debugGlobal;

/* ******************************************************************************************** */
typedef Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain joint values
typedef Matrix<double, 6, 6> Matrix6d;			///< A typedef for convenience to contain wrenches

/* ******************************************************************************************** */
// Globals for imu, motors and joystick

filter_kalman_t *kfImu;					///< the kalman filter to smooth the imu readings
filter_kalman_t *kfLWheel;		///< the kalman filter to smooth the pos/val readings of left wheel
filter_kalman_t *kfRWheel;	///< the kalman filter to smooth the pos/val readings of right wheel
ach_channel_t imuChan;				///< the state channel to listen to imu data
somatic_d_t daemon_cx;				///< The properties of this "daemon"
somatic_motor_t amc; 					///< The interface to the wheel motor group
somatic_motor_t waist;
somatic_motor_t llwa;
somatic_motor_t rlwa;
ach_channel_t js_chan;				///< The ach channel to the joystick daemon
ach_channel_t left_ft_chan;				///< The ach channel to the left ft 
ach_channel_t right_ft_chan;				///< The ach channel to the right ft

simulation::World* world;			///< the world representation in dart
SkeletonDynamics* robot;			///< the robot representation in dart

bool start = false;						///< Giving time to the user to get the robot in balancing angle
bool complyTorque = false;
bool resetFT = false; 		
	
double jsFwdAmp;				///< The gains for joystick forward/reverse input
double jsSpinAmp;				///< The gains for joystick left/right spin input

/* ******************************************************************************************** */
// All the freaking gains

extern size_t MODE;
extern Vector6d K_ground;
extern Vector2d J_ground;
extern Vector6d K_stand;
extern Vector2d J_stand;
extern Vector6d K_sit;
extern Vector2d J_sit;
extern Vector6d K_balLow;
extern Vector2d J_balLow;
extern Vector6d K_balHigh;
extern Vector2d J_balHigh;
extern Vector6d K;	

/* ******************************************************************************************** */
// The arm indices to set/get configurations from dart

extern std::vector <int> left_arm_ids;			///< Ids for left arm
extern std::vector <int> right_arm_ids;			///< Ids for right arm
extern std::vector <int> imuWaist_ids;			///< Ids for waist/imu

/// Makes the small 1e-17 values in a matrix zero for printing
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat);

/* ******************************************************************************************** */
// Constants for end-effector wrench estimation

static const double eeMass = 2.3 + 0.169 + 0.000;			///< The mass of the robotiq end-effector
static const Vector3d s2com (0.0, -0.008, 0.091); // 0.065 robotiq itself, 0.026 length of ext + 2nd

/* ******************************************************************************************** */
// Helper functions

/// Sets a global variable ('start') true if the user presses 's'
void *kbhit(void *);

/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick 
bool getJoystickInput(double& js_forw, double& js_spin);

/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState);

/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt, Vector3d* com = NULL, double* imu = NULL); 

/// Updates the dart robot representation
void updateDart (double imu);

/// Reads imu values from the ach channels and computes the imu values
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
		filter_kalman_t* kf);

/// Reads FT data from ach channels
bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data);

/// Computes the offset due to the weights of the end-effector in the FT sensor readings
void computeOffset (double imu, double waist, const somatic_motor_t& lwa, const Vector6d& raw, 
		SkeletonDynamics& robot, Vector6d& offset, bool left);

/// Givent the raw FT data, gives the wrench in the world frame acting on the FT sensor
void computeExternal (const Vector6d& input, SkeletonDynamics& robot, Vector6d& external, bool left);

/// Given the wrench at the FT sensor givers the wrench on the wheels
void computeWheelWrench(const Vector6d& wrenchSensor, SkeletonDynamics& robot, Vector6d& wheelWrench, bool left);

/// ........
void readGains();

/* ******************************************************************************************** */
// Useful macros

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define pv(a) std::cout << #a << ": " << fix((a).transpose()) << "\n" << std::endl
#define pc(a) std::cout << #a << ": " << (a) << "\n" << std::endl
#define pm(a) std::cout << #a << ":\n " << fix((a).matrix()) << "\n" << std::endl
#define pmr(a) std::cout << #a << ":\n " << fix((a)) << "\n" << std::endl
#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
	llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);
#define darm (cout << "dq: "<<llwa.vel[0] << ", " <<llwa.vel[1] << ", " << llwa.vel[2] << ", " << \
	llwa.vel[3] << ", " << llwa.vel[4] << ", " << llwa.vel[5] << ", " << llwa.vel[6] << endl);
#define eig7(x) (Vector7d() << (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5], (x)[6]).finished()

/* ******************************************************************************************** */
