/**
 * @file 04-impedance.cpp
 * @author Can Erdogan
 * @date June 29, 2013
 * @brief This executable demonstrates the manipulator follow a trajectory while being partially
 * compliant to external force (on Krang). The idea is that we will send positions to the motors
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
#include <fstream>

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
vector <VectorXd> path;           ///< The path that the robot will execute

const int r_id = 0;

/* ******************************************************************************************** */
/// Given a wrench, computes the joint space velocities so that wrench is minimized 
void wrenchToJointVels (const Vector6d& wrench, Vector7d& dq) {

	// Get the Jacobian towards computing joint-space velocities
	const char* nodeName = useLeftArm ? "lGripper" : "rGripper";
	static kinematics::BodyNode* eeNode = world->getSkeleton(r_id)->getNode(nodeName);
	MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jinv = Jt * (J * Jt).inverse();

	// Get the joint-space velocities by multiplying inverse Jacobian with the opposite wrench.
	dq = Jinv * wrench / 300.0;

	// Threshold the velocities
	for(size_t i = 0; i < 7; i++) {
		if(dq(i) > 0.1) dq(i) = 0.2;
		else if(dq(i) < -0.1) dq(i) = -0.2;
	}
}

/* ********************************************************************************************* */
void computeGoal (const Vector7d& traj, const Vector6d& wrench, Vector7d& goal) {

	// Make sure the threshold is not negligible
	static int i = 0;
	i++;
	if((i < 10) || ((wrench.topLeftCorner<3,1>().norm() < 7) && (wrench.bottomLeftCorner<3,1>().norm() < 0.4))) {
		goal = traj;
		return;
	}
	
	cout << "Felt wrench: " << wrench.transpose() << endl;

	// Compute the position offset due to the wrench - the effect should be set to make it sensitive
	// enough - we will normalize it if it is too much anyway
	static const double wrenchEffect = 100.0;
	static Vector7d dq;
	wrenchToJointVels(wrench, dq);
	Vector7d offset = dq * wrenchEffect;
	
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
	size_t c = 0, path_idx = 0;
	Vector6d raw, external;
	double imu = 0.0, waist = 0.0;
	while(!somatic_sig_received) {

		c++;

		// Check if reached a goal; if not update the motor readings
		if(path_idx == path.size() - 1) path_idx = 0;
		somatic_motor_update(&daemon_cx, &lwa);

		// Get imu/waist data
		getImu(&imu, imuChan);
		getWaist(&waist, waistChan);

		// Check that the current values are not being too much, give a warning if so
		for(size_t i = 0; i < 7; i++)
			if(fabs(lwa.cur[i]) > 8.0)
				printf("\t\t\tWARNING: Current at module %d has passed 8 amps: %lf amps\n", i, lwa.cur[i]);

		// Get the external force/torque values
		bool result = false;
		while(!result) result = getFT(daemon_cx, ft_chan, raw);
		computeExternal(imu, waist, lwa, raw + offset, *(world->getSkeleton(0)), external, useLeftArm);

		// Get the 7 dof arm value from the given 24dof path
		Vector7d pathGoal;
		for(size_t i = 10, j = 0; i < 23; i+=2, j++) pathGoal(j) = path[path_idx](i);
		pv(pathGoal);

		// Compute the next goal position
		Vector7d goal;
		computeGoal(pathGoal, external, goal);
		pv(goal);

		// Send the velocity commands 
		somatic_motor_cmd(&daemon_cx, &lwa, POSITION, goal.data(), 7, NULL);

		// Increment the position counter if we got close to our goal
		if((pathGoal - eig7(lwa.pos)).norm() < 1e-1) path_idx++;

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

/* ********************************************************************************************* */
/// Reads a joint data for the entire robot (24 dof - no grips) from a file to the global variable
void readFile () {

	// Open the file
	fstream file ("data");
	assert(file.is_open() && "Could not open the file!");

	// Read each line
	double temp;
	std::string line;
	VectorXd row = VectorXd::Zero(24);
	while(getline(file, line)) {
		size_t i = 0;
		stringstream stream (line, stringstream::in);
		while(stream >> temp) row(i++) = temp;
		path.push_back(row);
	}
}

/* ******************************************************************************************** */
/// The main thread
int main(const int argc, char** argv) {

	// Check if the user wants the right arm indicated by the -r flag
	if((argc > 1) && (strcmp(argv[1], "-r") == 0)) useLeftArm = false;

	// Read the trajectory from the data file in the build directory
	readFile();

	// Initialize the robot
	init(daemon_cx, js_chan, imuChan, waistChan, ft_chan, lwa, offset, useLeftArm);

	// Run and once done, halt motors and clean up
	run();
	destroy();
	return 0;
}

