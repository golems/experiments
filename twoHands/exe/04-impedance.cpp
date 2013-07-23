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

#include "helpers.h"

using namespace std;
using namespace dynamics;
using namespace simulation;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t imuChan;
ach_channel_t waistChan;				
Arm* larm = NULL, * rarm = NULL;	///< Arm descriptors with ft chan, somatic_motor_t and ft offset
vector <VectorXd> path;       ///< The path that the robot will execute
const int krang_id = 0;						///< The robot id in the simulation world
bool goInit = false;

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
	if(larm) 
		somatic_motor_cmd(&daemon_cx, &(larm->lwa), SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	if(rarm) 
		somatic_motor_cmd(&daemon_cx, &(rarm->lwa), SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_d_channel_close(&daemon_cx, &waistChan);
	somatic_d_channel_close(&daemon_cx, &imuChan);
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
void computeGoal (const Vector7d& traj, const Vector6d& wrench, Vector7d& goal, bool leftArm) {

	// Skip the first few steps
	static int i = 0;
	if(i++ < 1e6) {
		goal = traj;
		return;
	}

	// Make sure the threshold is not negligible
	if((wrench.topLeftCorner<3,1>().norm() < 7) && (wrench.bottomLeftCorner<3,1>().norm() < 0.4)) {
		goal = traj;
		return;
	}
	
	// Compute the position offset due to the wrench - the effect should be set to make it sensitive
	// enough - we will normalize it if it is too much anyway
	static const double wrenchEffect = 100.0;
	Vector7d dq;
	wrenchToJointVels(wrench, dq, leftArm);
	Vector7d offset = dq * wrenchEffect;
	
	// Normalize the offset if it is too much
	static const double maxOffset = 0.25;
	double offsetNorm = offset.norm();
	if(offsetNorm > maxOffset) offset = offset * (maxOffset / offsetNorm);

	// Add the offset to the goal
	goal = traj + offset;
}

/* ********************************************************************************************* */
/// Returns the external value for a given arm
void getExternal (Vector6d& external, bool leftArm) {

	// Get the data
	Arm* arm = leftArm ? larm : rarm;
	double imu = 0.0, waist = 0.0;
	static Vector6d raw;
	getImu(&imu, imuChan);
	while(!getWaist(&waist, waistChan));
	while(!getFT(daemon_cx, arm->ft_chan, raw));

	// Compute the external wrench
	computeExternal(imu, waist, arm->lwa, raw+arm->offset, *(world->getSkeleton(0)), external, 
		leftArm);
}

/* ********************************************************************************************* */
/// For a given arm, updates motor and wrench values, computes the goal and sends the data
void runArm(bool leftArm, const Vector7d& pathGoal) {

	// Update the arm and wrench values
	Vector6d external;
	Arm* arm = leftArm ? larm : rarm; 
	somatic_motor_update(&daemon_cx, &(arm->lwa));
	getExternal(external, leftArm);

	// Check that the current values are not being too much, give a warning if so
	for(size_t i = 0; i < 7; i++) {
		if(fabs(arm->lwa.cur[i]) > 12.0) {
			printf("\t\tDANGER: Mod. %d [%s] curr. over 12A: %lf, exitting!\n", i, 
				leftArm ? "l" : "r", arm->lwa.cur[i]);
			destroy();
			exit(0);
		}
		else if(fabs(arm->lwa.cur[i]) > 8.0)
			printf("\t\tWARNING: Mod. %d [%s] curr. over 8A: %lf\n", i, leftArm ? "l" : "r", 
				arm->lwa.cur[i]);
	}

	// Compute the goal
	Vector7d goal;
	computeGoal(pathGoal, external, goal, leftArm);

	// Send the position commands 
	somatic_motor_cmd(&daemon_cx, &(arm->lwa), POSITION, goal.data(), 7, NULL);
}

/* ********************************************************************************************* */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	int c = 0, path_idx = path.size()-1;
	Vector7d pathGoalLeft, pathGoalRight;
	while(!somatic_sig_received) {

		if(goInit) path_idx = path.size()-1;
		cout << "path_idx: " << path_idx << endl;

		// Get the motor goals for both arms from the path
		for(size_t i = 0; i < 7; i++) {
			pathGoalLeft(i) = path[path_idx](left_arm_ids[i]);
			pathGoalRight(i) = path[path_idx](right_arm_ids[i]);
		}

		pv(pathGoalLeft);
		pv(pathGoalRight);

		// Update the motor positions with the latest goal positions
		if(larm != NULL) runArm(true, pathGoalLeft);
		if(rarm != NULL) runArm(false, pathGoalRight);

		// Check if reached the goal; stop if reached the end
		double errorTolerance = goInit ? 1e-3 : 1e-3;
		double errorLeft = larm ? (pathGoalLeft - eig7(larm->lwa.pos)).norm() : 0.0;
		double errorRight = rarm ? (pathGoalRight - eig7(rarm->lwa.pos)).norm() : 0.0;
		if((errorLeft < errorTolerance) && (errorRight < errorTolerance)) path_idx--;
		if(path_idx < 0) break;
		usleep(1e5);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
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
	cout << "num points: " << path.size() << endl;
}

/* ******************************************************************************************** */
/// The main thread
int main(const int argc, char** argv) {

	// Check if the user wants the right arm indicated by the -r flag
	if((argc > 1) && (strcmp(argv[1], "-r") == 0)) rarm = new Arm();
	else if((argc > 1) && (strcmp(argv[1], "-b") == 0)) larm = new Arm(), rarm = new Arm();
	else larm = new Arm();

	// Check if the user wants the arms to go the initial point in the traj. with the -i flag
	if((argc > 2) && (strcmp(argv[2], "-i") == 0)) goInit = true;

	// Read the trajectory from the data file in the build directory
	readFile();

	// Initialize the robot
	init(daemon_cx, js_chan, imuChan, waistChan, larm, rarm);

	// Run and once done, halt motors and clean up
	run();
	destroy();
	return 0;
}

