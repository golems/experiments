/**
 * @file helpers.h
 * @author Can Erdogan
 * @date June 12, 2013
 * @brief This file contains some helper functions such as reading force/torque data if available.
 */

#pragma once

#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <syslog.h>
#include <fcntl.h>

#include <Eigen/Dense>

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include "kinematics.h"
#include "initModules.h"
#include "motion.h"
#include <iostream>

using namespace Eigen;

#define pv(x) std::cout << #x << ": " << (x).transpose() << std::endl;

typedef Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain joint values
typedef Matrix<double, 6, 6> Matrix6d;			///< A typedef for convenience to contain wrenches

static const double eeMass = 1.6 + 0.169 + 0.000;			///< The mass of the end-effector

/// Set the vector from the sensor origin to the gripper center of mass (m)
static const Vector3d s2com (0.0, 0.0, 0.09); // 0.0683 schunk itself, 0.026 length of ext + 2nd

/// Returns the representation of the end-effector frame in the base frame
void forwardKinematics (const somatic_motor_t& llwa, MatrixXd& Tbee);

/// Computes the initial offset from the given first raw value
void computeOffset (const somatic_motor_t& llwa, const Vector6d& raw, Vector6d& offset);

/// Computes the external force and torque from the values assuming that the input is already
/// corrected for the effect of gravity. That is the arm readings should reflect the weight
/// of the end-effector when there are no external inputs.
void computeExternal (const somatic_motor_t& llwa, const Vector6d& input, Vector6d& external);

/// Initializes the daemon, joystick/ft channels, left arm and computes the initial offset for ft
void init (somatic_d_t& daemon_cx, ach_channel_t& js_chan, ach_channel_t& ft_chan, 
		somatic_motor_t& llwa, Vector6d& offset);

/// Returns the f/t data if available at that instance
bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data);

