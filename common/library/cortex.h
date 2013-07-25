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
#include <ach.h>

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
		MODE_GRIPPERS = 32,
		MODE_ALL = MODE_AMC | MODE_LARM | MODE_RARM | MODE_TORSO | MODE_WAIST | MODE_GRIPPERS,
	};

	/// Initializes the interfaces to the motor groups based on the given hardware mode
	Hardware (Mode mode, somatic_d_t* daemon_cx); 

public:
	// The interfaces to the sensors on Krang

	ach_channel_t* imu;										///< Imu sensor for balance angle
	FT* lft;															///< Force/torque data from left arm
	FT* rft;															///< Force/torque data from right arm

public:
	// The interfaces to the motor groups on Krang

	somatic_motor_t* amc;									///< Wheels motor interface
	somatic_motor_t* larm;								///< Left arm motor interface
	somatic_motor_t* rarm;								///< Right arm motor interface
	somatic_motor_t* torso;								///< Torso motor interface
	somatic_motor_t* grippers;						///< Wheels motor interface
	ach_channel_t* waistCmdChan;					///< Command channel for the waist daemon
	ach_channel_t* waistState;						///< State channel for the waist modules
};

/* ******************************************************************************************** */
// Helper functions to initialize modules and sensors

/// Initializes a motor group that is represented with a somatic structure
void initMotorGroup (somatic_d_t* daemon_cx, somatic_motor_t*& motors, const char* cmd_name,
		const char* state_name, VectorXd minPos, VectorXd maxPos, VectorXd minVel, VectorXd maxVel);

/// 






