/**
 * @file sensors.h
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The header file for the sensor class and helper functions associated with Krang.
 */

#pragma once

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <imud.h>
#include <filter.h>
#include <ach.h>

#include <Eigen/Dense>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/BodyNode.h>

#include "util.h"

namespace Krang {

/* ******************************************************************************************** */
class FT {
public:

	/// Indicates which type of gripper is after the f/t sensor which affects the mass and com
	enum GripperType { GRIPPER_TYPE_ROBOTIQ = 0, GRIPPER_TYPE_SCHUNK, GRIPPER_TYPE_NONE };

	/// The constructor. The type of the gripper after f/t affects the readings, robot + side are
	/// used to estimate the frame of the f/t in the robot base frame.
	FT (GripperType type, somatic_d_t* daemon_cx, dynamics::SkeletonDynamics* robot, Side side);
	bool getRaw(Vector6d& raw);  					///< Gets the latest raw reading from the f/t channel
	void updateExternal();								///< Updates estimate for the external input 

	/// Computes the error between ideal and expected readings assuming no external forces
	/// This is used in the beginning to compute an offset when we "know" we have no external forces
	/// and also to estimate the external forces. Note that the output is in sensor frame!
	void error(const Vector6d& reading, Vector6d& error);	

public:
	// Variables to compensate for the weight of the gripper

	/// The offset from the raw readings for the ideal readings. An ideal reading still has the
	/// weight of the gripper and any other external weights in it.
	Vector6d offset;	
	Vector6d lastExternal;			///< The last estimate for the external force/torque input
	double gripperMass;					///< The mass of the objects after the f/t sensor
	Vector3d gripperCoM;				///< The center of mass of the objects after the f/t sensor

public:
	// Variables to determine the kinematics that affect the compensation and the data channel

	Side side;															///< Indices the left or the right f/t sensor
	dynamics::SkeletonDynamics* robot;			///< The kinematics of the robot
	somatic_d_t* daemon_cx;
	ach_channel_t* chan;										///< The data channel
};

/* ******************************************************************************************** */
// Helper functions to update other sensors (i.e. imu, kinect..)

/// Returns the imu value and filters it if a filter struct is given
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
	     filter_kalman_t* kf);


};	// end of namespace
