/**
 * @file 05-jInv.cpp
 * @author Can Erdogan
 * @date June 18, 2013
 * @brief This executable demonstrates the use of the inverse Jacobian to follow a trajectory in 
 * with the real robot using dart data structures.
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

#define pq(a) std::cout << #a << ": " << (a) << std::endl
#define pmr(a) std::cout << #a << ":\n " << ((a)) << "\n" << std::endl

#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
	llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);

#define NUM_GOALS 3
const size_t r_id = 4;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;
World* mWorld = NULL;

/* ********************************************************************************************* */
/// Given a workspace velocity, returns the joint space velocity
VectorXd workToJointVelocity (kinematics::BodyNode* eeNode, const VectorXd& xdot) {

	// Get the Jacobian towards computing joint-space velocities
	MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jinv = Jt * (J * Jt).inverse();

	// Get the joint-space velocities by multiplying inverse Jacobian with x.
	VectorXd qdot = Jinv * xdot;
	return qdot;
}

/* ********************************************************************************************* */
/// Returns the workspace velocity to get the end-effector to the desired goal transformation
/// If already there, returns true.
bool workVelocity (World* mWorld, kinematics::BodyNode* eeNode, size_t currGoal, VectorXd& xdot) {

	// Get the current goal location and orientation (as a quaternion);
	MatrixXd goalTransform = mWorld->getSkeleton(currGoal)->getRoot()->getWorldTransform();
	VectorXd goalPos = goalTransform.topRightCorner<3,1>();
	Quaternion <double> goalOri (goalTransform.topLeftCorner<3,3>());

	// Get the current end-effector location and orientation (as a quaternion);
	Matrix4d axisChange;
//	axisChange << 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	MatrixXd eeTransform = eeNode->getWorldTransform(); // * axisChange;
	VectorXd eePos = eeTransform.topRightCorner<3,1>();
	Quaternion <double> eeOri (eeTransform.topLeftCorner<3,3>());

	// Find the position error
	VectorXd errPos = goalPos - eePos;
	pv(errPos);
	
	// Find the orientation error and express it in RPY representation
	Quaternion <double> errOriQ = goalOri * eeOri.inverse();
	Matrix3d errOriM = errOriM = errOriQ.matrix();
	Vector3d errOri = math::matrixToEuler(errOriM, math::XYZ);
	pv(errOri);

	// Check if the goal is reached
	static const double posLimit = 0.01;
	static const double oriLimit = 0.01;
	if((errPos.norm() < posLimit) && (errOri.norm() < oriLimit)) return true;

	// Get the workspace velocity
	static const double kSpeed = 0.07;
	xdot = VectorXd (6);
	xdot << errPos, errOri;
	xdot = kSpeed * xdot.normalized();
	return false;
}

/* ********************************************************************************************* */
/// Detects collisions with the environment and within robot
void run() {

	static size_t currGoal = 0;
	static size_t reachWaitIters = 0;

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, move towards the goal(s)
	size_t c = 0;
	VectorXd vals (7);
	while(!somatic_sig_received) {
		
		c++;
		cout << "goal: " << currGoal << endl;

		// Update the arm position in dart with the real arm positions
		somatic_motor_update(&daemon_cx, &llwa);
		for(size_t i = 0; i < 7; i++) vals(i) = llwa.pos[i];
		vector <int> arm_ids;
		for(size_t i = 4; i < 17; i+=2) arm_ids.push_back(i + 6);  
		mWorld->getSkeleton(r_id)->setConfig(arm_ids, vals);
		
		// Get the workspace velocity
		kinematics::BodyNode* eeNode = mWorld->getSkeleton(r_id)->getNode("lgPlate1");
		VectorXd xdot;
		bool reached = workVelocity(mWorld, eeNode, currGoal, xdot);
		if(reached) {

			// If we are within the proximity of a goal for 30 frames, we decide that we've reached it
			cout << "Reached goal!\n";
			if(reachWaitIters < 30) reachWaitIters++;
			else {
				reachWaitIters = 0;
				currGoal = min((size_t) NUM_GOALS, currGoal+1);
			}

			// Rerun to increment the timer and/or to set the next goal
			double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
			continue;
		}

		// Get the joint-space velocities by multiplying inverse Jacobian with x.
		VectorXd qdot = workToJointVelocity(eeNode, xdot);

		// Limit the joint values to +- 0.2 
		for(size_t i = 0; i < 7; i++) {
			if(qdot(i) > 0.1) qdot(i) = 0.2;
			else if(qdot(i) < -0.1) qdot(i) = -0.2;
		}
				
		// Apply the joint space velocities to the robot
		bool run = 1;
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		for(size_t i = 0; i < 7; i++) dq[i] = qdot(i);
		if(run)
			somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);

		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void init() {

	// ============================================================================
	// Load the robot model and set the timer

	// Load the world
	DartLoader dl;
	mWorld = dl.parseWorld("../scenes/02-World-JInv.urdf");
	assert((mWorld != NULL) && "Could not find the world");

	// ============================================================================
	// Initialize robot stuff

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "04-gripperWeight";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the left arm
	initArm(daemon_cx, llwa, "llwa");

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}
