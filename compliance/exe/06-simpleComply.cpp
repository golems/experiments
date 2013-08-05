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
 * @file 06-compliance.cpp
 * @author Can Erdogan
 * @date June 19, 2013
 * @brief This file demonstrates the full compliance motion where the correct input f/t readings,
 * estimate the external f/t values, threshold them to not deal with the modeling error of the
 * sensor, get them in the world frame and move towards the opposite direction using inverse 
 * Jacobians.
 * TODO Limit acceleration to avoid jerk, also frame rate
 * TODO Change nominal acceleration based on average force
 */

#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include "helpers.h"
#include "initModules.h"
#include "motion.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;
using namespace simulation;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t imuChan;
ach_channel_t waistChan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;
Vector6d offset;							///< the offset we are going to decrease from raw readings
bool useLeftArm = false;				///< The indicator that the left arm will be used for this program

const int krang_id = 0;

bool do_run = true;

/* ******************************************************************************************** */
/// Given a wrench, computes the joint space velocities so that wrench is minimized 
void wrenchToJointVels (const Vector6d& wrench, Vector7d& dq) {

	// Get the Jacobian towards computing joint-space velocities
	const char* nodeName = useLeftArm ? "lGripper" : "rGripper";
	static kinematics::BodyNode* eeNode = world->getSkeleton(krang_id)->getNode(nodeName);
	MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jinv = Jt * (J * Jt).inverse();

	// Get the joint-space velocities by multiplying inverse Jacobian with the opposite wrench.
	dq = Jinv * wrench / 300.0;
	//pv(dq);

	// Threshold the velocities
	for(size_t i = 0; i < 7; i++) {
		if(dq(i) > 0.1) dq(i) = 0.2;
		else if(dq(i) < -0.1) dq(i) = -0.2;
	}
}

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	Vector6d raw, external;
	Vector7d dq, vals;
	vector <int> arm_ids;
	double waist = 0.0, imu = 0.0;	
	for(size_t i = 4; i < 17; i+=2) arm_ids.push_back(i + 6);  
	double dqZero [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	cout << "\n\nStarting the loop" << endl;
	while(!somatic_sig_received) {
		
		c++;

		// Update the arm position in dart with the real arm positions
		somatic_motor_update(&daemon_cx, &llwa);
		for(size_t i = 0; i < 7; i++) vals(i) = llwa.pos[i];
		world->getSkeleton(krang_id)->setConfig(arm_ids, vals);

		// Get imu/waist data
		getImu(&imu, imuChan);
		getWaist(&waist, waistChan);
		
		// Get the f/t sensor data and compute the ideal value
		size_t k = 1e1;
		bool result = (c % k == 0) && getFT(daemon_cx, ft_chan, raw);
		if(!result) continue;
		
		// Compute the ideal value
		Vector6d ideal = raw + offset;

		// Compute the external forces from ideal readings and move it to bracket frame
		computeExternal(imu, waist, llwa, ideal, *(world->getSkeleton(krang_id)), external, useLeftArm);
                
		// Threshold the values - 4N for forces, 0.4Nm for torques
		if((external.topLeftCorner<3,1>().norm() < 7) && 
                   (external.bottomLeftCorner<3,1>().norm() < 0.4)) {
                    if (do_run)
			somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dqZero, 7, NULL);
                    continue;
		}

		// Increase the sensitivity to the torque values
		external(3) *= 20;
		external(4) *= 20;
		external(5) *= 40;
		pv(external);

		// Compute the necessary motion in the joint space by multiply the -external with Jacobian inv
		wrenchToJointVels(external, dq);
		pv(dq);

		// Apply the joint space velocities to the robot
		if(do_run)
			somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq.data(), 7, 
				NULL);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
    if(do_run)
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
    somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// // Check if the user wants the right arm indicated by the -r flag
	// if((argc > 1) && (strcmp(argv[1], "-r") == 0)) useLeftArm = false;

	// Initialize the robot
	init(daemon_cx, js_chan, imuChan, waistChan, ft_chan, llwa, offset, useLeftArm);

	// Run and once done, halt motors and clean up
	run();
	destroy();
	return 0;
}
