/**
 * @file sensors.h
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The header file for the sensor class and helper functions associated with Krang.
 */

#pragma once

#include "kinematics.h"

typedef Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values

namespace Krang {

/* ******************************************************************************************** */
class FT {
public:

	/// Indicates which type of gripper is after the f/t sensor which affects the mass and com
	enum GripperType {
		GRIPPER_TYPE_ROBOTIQ = 0,
		GRIPPER_TYPE_SCHUNK,
		GRIPPER_TYPE_NONE
	};

	/// The constructor. The type of the gripper after f/t affects the readings, robot + side are
	/// used to estimate the frame of the f/t in the robot base frame.
	FT (GripperType type, dynamics::SkeletonDynamics* robot, Side side);

public:
	// Variables to compensate for the weight of the gripper and the data channel

	/// The offset from the raw readings for the ideal readings. An ideal reading still has the
	/// weight of the gripper and any other external weights in it.
	Vector6d offset;	
	double gripperMass;					///< The mass of the objects after the f/t sensor
	Vector3d gripperCoM;				///< The center of mass of the objects after the f/t sensor
	ach_channel_t* chan;				///< The data channel

public:
	// Variables to determine the kinematics that affect the compensation

	Side side;															///< Indices the left or the right f/t sensor
	dynamics::SkeletonDynamics* robot;			///< The kinematics of the robot
};

/* ******************************************************************************************** */
// Helper functions to update other sensors (i.e. imu, kinect..)

/// Returns the imu value and filters it if a filter struct is given
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
		filter_kalman_t* kf);

};
