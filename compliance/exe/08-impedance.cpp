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

#include "helpers.h"
#include <Eigen/StdVector>

using namespace std;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;
World* mWorld = NULL;					///< Dart structure that contains the robot kinematics
Vector6d offset;							///< the offset we are going to decrease from raw readings
vector <Vector7d, aligned_allocator<Vector7d> > traj;	///< The traj to follow in joint space pos

/* ******************************************************************************************** */
/// Given a wrench, computes the joint space velocities so that wrench is minimized 
void wrenchToJointVels (const Vector6d& wrench, Vector7d& dq) {

	// Get the Jacobian towards computing joint-space velocities
	static kinematics::BodyNode* eeNode = mWorld->getSkeleton(r_id)->getNode("lGripper");
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

/* ********************************************************************************************* */
void computeGoal (const Vector7d& traj, const Vector6d& wrench, Vector7d& goal) {

	// Make sure the threshold is not negligible
	if((wrench.topLeftCorner<3,1>().norm() < 7) && (wrench.bottomLeftCorner<3,1>().norm() < 0.4)) {
		goal = traj;
		return;
	}

	// Compute the position offset due to the wrench - the effect should be set to make it sensitive
	// enough - we will normalize it if it is too much anyway
	static const double wrenchEffect = 100.0;
	static Vector7d dq;
	wrenchToJointVels(external, dq);
	Vector7d offset = dq * wrenchEffect;
	
	// Normalize the offset if it is too much
	static const double maxOffset = 1.0;
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

	// If we are going to follow a trajectory, it should not be empty
	assert((recordTraj || !traj.empty()) && "The trajectory should not be empty if no joystick.");

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0, traj_idx = 0;
	Vector6d raw;
	while(!somatic_sig_received) {

		c++;

		// Check if reached a goal; if not update the motor readings
		if(traj_idx == traj.size() - 1) break;
		somatic_motor_update(&daemon_cx, &llwa);

		// Get the external force/torque values
		bool result = false;
		while(!result) result = getFT(daemon_cx, ft_chan, raw);
		computeExternal(llwa, raw + offset, *(mWorld->getSkeleton(0)), external);

		// Compute the next goal position
		Vector7d goal;
		computeGoal(traj[traj_idx], external);

		// Send the velocity commands 
		somatic_motor_cmd(&daemon_cx, &llwa, POSITION, goal, 7, NULL);

		// Increment the position counter if we got close to our goal
		if((traj[traj_idx] - eig7(llwa.pos)).norm() < 1e-1) traj_idx++;

		usleep(1e6 * dt);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
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

	// Load the world
	DartLoader dl;
	mWorld = dl.parseWorld("../scenes/01-World-Robot.urdf");
	assert((mWorld != NULL) && "Could not find the world");

	// Create a trajectory
	double q [] = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0};	
	for(double i = M_PI_2; i >= -0.002; i -= 0.01) {
		q[4] = i;
		traj.push_back(eig7(q));	
	}

	// Initialize the robot
	init(daemon_cx, js_chan, ft_chan, llwa, offset);

	// Run and once done, halt motors and clean up
	run();
	destroy();
	return 0;
}

