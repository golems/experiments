/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file kore.h
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The main header file for the "K"rang "O"peration "R"untime "E"nvironment 
 */

#pragma once

#include "sensors.h"
#include "safety.h"

using namespace Eigen;

namespace Krang {

/* ******************************************************************************************** */
/// The interface to the active motor groups on the robot.
class KHardware {
public:

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
	KHardware (Mode mode, somatic_d_t* daemon_cx, dynamics::SkeletonDynamics* robot); 

	/// The destructor which sends halt messages to all Schunk modules and 0-velocities to wheels
	~KHardware ();

public:
	// Updates the sensor readings and the kinematic structure

	/// Updates the sensor readings
	void updateSensors(double dt);

	/// Prints the state
	void printState();
	void printStateCurses(int row, int col);

private:	

	/// Updates the Dart's kinematics data structures with the latest readings
	/// This is made private because updateSensors already calls this. A user should not need to call
	/// it.
	void updateKinematics();

public:
	// Initializes the modules and sensors

	/// Initializes a motor group that is represented with a somatic structure
	void initMotorGroup (somatic_motor_t*& motors, const char* cmd_name, const char* state_name, 
	                     Eigen::VectorXd minPos, Eigen::VectorXd maxPos,
	                     Eigen::VectorXd minVel, Eigen::VectorXd maxVel);

	/// Initializes the amc wheels while using the average imu to create an offset (for absolute 
	/// wheel positions)
	void initWheels ();

	/// Initializes the imu channel, the filter and averages first 500 readings for correct 
	/// wheels and f/t offsets
	void initImu ();

	/// Initializes the waist module group: (1) first creates the somatic interface without a command
	/// channel to get updates, (2) second, creates a channel to the waist daemon
	void initWaist ();

public:
	// The fields to keep track of the daemon context, mode and the robot kinematics
	
	Mode mode;														///< Indicates which motor groups are used
	somatic_d_t* daemon_cx;								///< The daemon context for the running program
	dynamics::SkeletonDynamics* robot;		///< The kinematics of the robot

public:
	// The interfaces to the sensors on Krang

	ach_channel_t* imu_chan;							///< Imu sensor for balance angle
	double imu, imuSpeed;									///< Latest imu readings (or the mean over a window)
	filter_kalman_t* kfImu; 							///< The kalman filter for the imu readings

	FT* lft;															///< Force/torque data from left arm
	FT* rft;															///< Force/torque data from right arm

public:
	// The interfaces to the motor groups on Krang

	somatic_motor_t* amc;									///< Wheel motors interface
	somatic_motor_t* larm;								///< Left arm motors interface
	somatic_motor_t* rarm;								///< Right arm motors interface
	somatic_motor_t* torso;								///< Torso motor interface
	somatic_motor_t* lgripper;						///< Gripper motors interface
	somatic_motor_t* rgripper;						///< Gripper motors interface
	somatic_motor_t* waist;								///< Waist motors interface - only read
	ach_channel_t* waistCmdChan;					///< Command channel for the waist daemon
};

};	// end of namespace
