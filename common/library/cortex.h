/**
 * @file cortex.h
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The main header file for the Krang support library. 
 */

#pragma once

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <filter.h>
#include <ach.h>

#include "kinematics.h"
#include "sensors.h"

namespace Krang {

/* ******************************************************************************************** */
/// The interface to the active motor groups on the robot.
class Hardware {
public:

	/// The indicator for the left or right side	
	enum Side {
		LEFT = 0,
		RIGHT
	};

	/// The indicators for the motor groups to be used
	enum Mode {
		MODE_AMC = 1,
		MODE_LARM = 2,
		MODE_RARM = 4,
		MODE_TORSO = 8,
		MODE_WAIST = 16,
		MODE_GRIPPERS = 32,													///< Indicates the robotiq grippers (default)
		MODE_GRIPPERS_SCH = 64,											///< Indicates the schunk grippers
		MODE_ALL = MODE_AMC | MODE_LARM | MODE_RARM | MODE_TORSO | MODE_WAIST | MODE_GRIPPERS,
		MODE_ALL_GRIPSCH = MODE_AMC | MODE_LARM | MODE_RARM | MODE_TORSO | MODE_WAIST | MODE_GRIPPERS_SCH
	};

	/// Initializes the interfaces to the motor groups based on the given hardware mode
	Hardware (Mode mode, somatic_d_t* daemon_cx, SkeletonDynamics* robot); 

	/// Updates the sensor readings
	void updateSensors(double dt);

	/// Updates the Dart's kinematics data structures with the latest readings
	void updateKinematics();

public:
	// The fields to keep track of the daemon context, mode and the robot kinematics
	
	Mode mode;														///< Indicates which motor groups are used
	somatic_d_t& daemon_cx;								///< The daemon context for the running program
	SkeletonDynamics* robot;							///< The kinematics of the robot

public:
	// The interfaces to the sensors on Krang

	ach_channel_t* imu;										///< Imu sensor for balance angle
	double imu, imuSpeed;									///< Latest imu readings (or the mean over a window)
	kalman_filter_t* kfImu; 							///< The kalman filter for the imu readings

	FT* lft;															///< Force/torque data from left arm
	FT* rft;															///< Force/torque data from right arm

public:
	// The interfaces to the motor groups on Krang

	somatic_motor_t* amc;									///< Wheel motors interface
	somatic_motor_t* larm;								///< Left arm motors interface
	somatic_motor_t* rarm;								///< Right arm motors interface
	somatic_motor_t* torso;								///< Torso motor interface
	somatic_motor_t* grippers;						///< Gripper motors interface
	somatic_motor_t* waist;								///< Waist motors interface - only read
	ach_channel_t* waistCmdChan;					///< Command channel for the waist daemon
};

/* ******************************************************************************************** */
// Helper functions to initialize modules and sensors

/// Initializes the imu channel, the filter and averages first 500 readings for correct 
/// wheels and f/t offsets
void initImu (somatic_d_t* daemon_cx, ach_channel_t* imu, double& imu, double& imuSpeed, 
		kalman_filter_t*& kfImu);

/// Initializes a motor group that is represented with a somatic structure
void initMotorGroup (somatic_d_t* daemon_cx, somatic_motor_t*& motors, const char* cmd_name,
		const char* state_name, VectorXd minPos, VectorXd maxPos, VectorXd minVel, VectorXd maxVel);

/// Initializes the amc wheels while using the average imu to create an offset (for absolute wheel
/// positions).
void initWheels (somatic_d_t* daemon_cx, somatic_motor_t*& motors, double imu);

};
