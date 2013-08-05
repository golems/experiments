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
 * @file 08-impedance.cpp
 * @author Can Erdogan
 * @date June 23, 2013
 * @brief This executable demonstrates the manipulator follow a trajectory while being partially
 * compliant to external force. The idea is that we will send position commands to the motors
 * which incorporate both the trajectory and the f/t sensor information. To do so, we will only
 * consider f/t readings that have a norm > 5N and if there is such a reading, we will move the
 * goal position with vector v such that the f/t value is minimized. If the norm of v is more
 * than some radius r, we will normalize it so that we don't diverge from the path too much.
 */

#include <Eigen/StdVector>
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
somatic_motor_t lwa;
Vector6d offset;							///< the offset we are going to decrease from raw readings
vector <Vector7d, aligned_allocator<Vector7d> > traj;	///< The traj to follow in joint space pos
bool useLeftArm = true;				///< The indicator that the left arm will be used for this program

const int krang_id = 0;

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
	vector <int> dofs;
	for(size_t i = 0; i < 24; i++) dofs.push_back(i);
	//cout << "\nq in wrenchTo..: " << world->getSkeleton(r_id)->getConfig(dofs).transpose() << endl;
	cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << endl;
	cout << "\nJ: \n" << J << endl;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jinv = Jt * (J * Jt).inverse();
	cout << "\n\nJinv: \n" << Jinv << endl;

	// Get the joint-space velocities by multiplying inverse Jacobian with the opposite wrench.
	pv(wrench);
	dq = Jinv * wrench / 300.0;
	pv(dq);

	// Threshold the velocities
	double limit = 0.2;
	for(size_t i = 0; i < 7; i++) {
		if(dq(i) > limit) dq(i) = limit;
		else if(dq(i) < -limit) dq(i) = -limit;
	}
}

/* ********************************************************************************************* */
void computeGoal (const Vector7d& traj, const Vector6d& wrench, Vector7d& goal) {

	// Make sure the threshold is not negligible
	if((wrench.topLeftCorner<3,1>().norm() < 7) && (wrench.bottomLeftCorner<3,1>().norm() < 0.4)) {
		goal = traj;
		return;
	}
	
	cout << "\nFelt wrench: " << wrench.transpose() << endl;

	// Compute the position offset due to the wrench - the effect should be set to make it sensitive
	// enough - we will normalize it if it is too much anyway
	static const double wrenchEffect = 100.0;
	static Vector7d dq;
	wrenchToJointVels(wrench, dq);
	pv(dq);
	Vector7d offset = dq * wrenchEffect;
	pv(offset);
	
	// Normalize the offset if it is too much
	static const double maxOffset = 0.25;
	double offsetNorm = offset.norm();
	if(offsetNorm > maxOffset) offset = offset * (maxOffset / offsetNorm);

	// Add the offset to the goal
	goal = traj + offset;
}

/* ********************************************************************************************* */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0, traj_idx = 0;
	Vector6d raw, external;
	double imu = 0.0, waist = 0.0;
	while(!somatic_sig_received) {

		c++;

		// Check if reached a goal; if not update the motor readings
		if(traj_idx >= traj.size() - 1) traj_idx = 0; //break;
		somatic_motor_update(&daemon_cx, &lwa);
		traj_idx = 0;

		// Get imu/waist data
		getImu(&imu, imuChan);
		while(!getWaist(&waist, waistChan));

		// Check that the current values are not being too much, give a warning if so
		for(size_t i = 0; i < 7; i++)
			if(fabs(lwa.cur[i]) > 8.0)
				printf("\t\t\tWARNING: Current at module %d has passed 8 amps: %lf amps\n", i, lwa.cur[i]);

		// Get the external force/torque values
		bool result = false;
		while(!result) result = getFT(daemon_cx, ft_chan, raw);
		printf("imu: %lf, waist: %lf\n", imu, waist);
		computeExternal(imu, waist, lwa, raw + offset, *(world->getSkeleton(0)), external, useLeftArm);
		external = Vector6d();
		external << 0.0, -8.0, 0.0, 0.0, 0.0, 0.0;

		// Compute the next goal position
		Vector7d goal;
		computeGoal(traj[traj_idx], external, goal);

		// Send the velocity commands 
		somatic_motor_cmd(&daemon_cx, &lwa, POSITION, goal.data(), 7, NULL);

		// Increment the position counter if we got close to our goal
		if((traj[traj_idx] - eig7(lwa.pos)).norm() < 1e-1) traj_idx++;

		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
	somatic_motor_cmd(&daemon_cx, &lwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_d_channel_close(&daemon_cx, &waistChan);
	somatic_d_channel_close(&daemon_cx, &imuChan);
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main(const int argc, char** argv) {

	// Check if the user wants the right arm indicated by the -r flag
	if((argc > 1) && (strcmp(argv[1], "-r") == 0)) useLeftArm = false;

	// Create a trajectory moving j6 left and right 2*pi each time
/*
	double M_60 = M_PI / 3;
	double sign = useLeftArm ? 1.0 : -1.0;
	double q [] = {1.30 * sign, -M_60 * sign, 0.0, -M_60 * sign, 0.0, M_60/2 * sign, 0.0};	
	for(q[6] = 2*M_PI; q[6] >= -2*M_PI; q[6] -= 0.01) traj.push_back(eig7(q));	
	for(q[6] = -2*M_PI; q[6] < 2*M_PI; q[6] += 0.01) traj.push_back(eig7(q));	
*/

	// Initialize the robot
	init(daemon_cx, js_chan, imuChan, waistChan, ft_chan, lwa, offset, useLeftArm);

	Vector7d bla;
	bla << 0.555154,     -1.56964 ,  -1.5674 , 1.61075   , -0.00250535, -1.41014,  -1.55926;
	traj.push_back(bla);

	// Run and once done, halt motors and clean up
	run();
	destroy();
	return 0;
}

