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
 * @file sensors.h
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The header file for the sensor class and helper functions associated with Krang.
 */

#pragma once

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
	void error(const Vector6d& reading, Vector6d& error, bool inWorldFrame);	

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
class SpaceNav {
public:
	// functions

	/// The constructor. Opens ach channel and initializes the cache.
	SpaceNav(somatic_d_t* daemon_cx, char* chan_name, double cache_timeout);

	/// The destructor. Closes ach channel.
	~SpaceNav();

	/// Gets a safe, clean reading from the spacenav. If the network
	/// hiccups slightly or you call it too often, it returns a cached
	/// value from the last successful input. If it's unable to get a
	/// value for longer than cache_timout seconds, it assumes that
	/// the network has actually died and returns a nice, safe zero
	/// input.
	/// TODO Change the interface to cover buttons as well
	Eigen::VectorXd updateSpaceNav();

public:
	/// Get a new reading off the spacenav. Returns false if there is
	/// no new value to be had.
	bool getSpaceNavRaw(Eigen::VectorXd& spacenav_input_raw);

	int buttons [2];											///< Indicates whether a button is pressed		
	Eigen::VectorXd last_spacenav_input; 	///< Cache the last successful reading.
	double cache_timeout; 	///< How long we keep the cached value before assuming a disconnect
	double time_last_input; ///< When we last got a good reading
	somatic_d_t* daemon_cx; ///< Holds on to the daemon context so we can do ach channel things
	ach_channel_t spacenav_chan; 					///< The actual ach channel
};

/* ******************************************************************************************** */
// Helper functions to update other sensors (i.e. imu, kinect..)

/// Returns the imu value and filters it if a filter struct is given
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
	 filter_kalman_t* kf);

};	// end of namespace
