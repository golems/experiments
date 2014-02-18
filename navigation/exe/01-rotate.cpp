/**
 * @file 01-rotate.cpp
 * @author Can Erdogan
 * @date Feb 13, 2014
 * @brief Demonstrates the autonomous turn-in-place behavior. The robot stands up, turns 90
 * degrees left or right, and sits down, using encoder values.
 * Note that for the robot to turn th degrees in place, each wheel needs to do (th * R / r)
 * radians of rotation. R is the radius of rotation and r is the wheel radius.
 */


#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <kore.hpp>

#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <initModules.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>

 

Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) { return mat; }

typedef Eigen::Matrix<double,6,1> Vector6d;

using namespace std;

somatic_d_t daemon_cx;				///< The context of the current daemon
Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
simulation::World* world;			///< the world representation in dart
dynamics::SkeletonDynamics* robot;			///< the robot representation in dart

static const double wheelRadius = 0.264;	// r, cm
static const double rotationRadius = 0.350837; // R, cm
Vector6d K_stand, K_bal, K_turn_left, K_turn_right;
Vector6d state0;	///< in the beginning of entering mode 2
bool start = false;
bool dbg = false;
bool shouldRead = false;
double extraSpin = 0.0;
size_t integralWindow = 0;
double Ksint, intErrorLimit;
double offsetAngle = 0.0;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
size_t mode = 0;		// 0 sitting, 1 standing up, 2 stable, 3 turning, 4 sitting down

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	// Get the gains
	Vector6d* kgains [] = {&K_stand, &K_bal, &K_turn_left, &K_turn_right};
	ifstream file ("/home/cerdogan/Documents/Software/project/krang/experiments/navigation/data/gains-01.txt");
	assert(file.is_open());
	char line [1024];
	for(size_t k_idx = 0; k_idx < 4; k_idx++) {
		*kgains[k_idx] = Vector6d::Zero();
		file.getline(line, 1024);
		std::stringstream stream(line, std::stringstream::in);
		size_t i = 0;
		double newDouble;
		while ((i < 6) && (stream >> newDouble)) (*kgains[k_idx])(i++) = newDouble;
	}

	// Get the integral control options
	file.getline(line, 1024);
	std::stringstream stream(line, std::stringstream::in);
	stream >> integralWindow;
	file.getline(line, 1024);
	std::stringstream stream2(line, std::stringstream::in);
	stream2 >> Ksint;
	file.getline(line, 1024);
	std::stringstream stream3(line, std::stringstream::in);
	stream3 >> intErrorLimit;
	file.close();
	printf("READ GAINS!\n");
	printf("integralWindow: %d, Ksint: %lf, intErrorLimit: %lf\n", 
		integralWindow, Ksint, intErrorLimit);
}

/* ********************************************************************************************* */
/// Get the mode input
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		pthread_mutex_lock(&mutex);
		if(input=='0') mode = 0;
		else if(input=='1') mode = 1;
		else if(input=='2') mode = 2;
		else if(input=='3') mode = 3;
		else if(input=='4') mode = 4;
		else if(input=='5') mode = 5;
		else if(input=='k') extraSpin += 1.0;
		else if(input=='j') extraSpin += -1.0;
		else if(input=='s') start = !start;
		else if(input=='r') shouldRead = true;
		else if(input=='h') offsetAngle += 0.3;
		else if(input=='l') offsetAngle -= 0.3;
		pthread_mutex_unlock(&mutex);
	}
}

/* ******************************************************************************************** */
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "01-rotate";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot); 

	// Check that the waist is at the expected angle
	assert(fabs(krang->waist->pos[0] - 2.86) < 0.05 && "The gains and offsets are set for 164 deg");
	
	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt) {

	// Read motor encoders, imu and ft and update dart skeleton
  krang->updateSensors(dt);

	// Calculate the COM	
	Eigen::Vector3d com = robot->getWorldCOM();
	com(2) -= 0.264;
	com(0) -= 0.0018;
	if(dbg) cout << "com: " << com.transpose() << endl;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2));
	state(1) = krang->imuSpeed;
	state(2) = (krang->amc->pos[0] + krang->amc->pos[1])/2.0 + krang->imu;
	state(3) = (krang->amc->vel[0] + krang->amc->vel[1])/2.0 + krang->imuSpeed;
	state(4) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	state(5) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;
}

/* ******************************************************************************************** */
void switchModes (const Vector6d& state) {
	static int balancedCounter = 0;
	if((mode == 5) && (krang->imu < -1.82)) mode = 0;
	else if(mode == 1) {
		if(fabs(state(0)) < 0.054) balancedCounter++;
		if(balancedCounter > 100) {
			mode = 2;
			balancedCounter = 0;
			// state0 = state;
		}	
	}
}

/* ******************************************************************************************** */
void computeTorques (const Vector6d& state, double& ul, double& ur) {

	// Initialize the integral controller
	static vector <double> spinErrors;
	static int spinIdx = 0;
	if((spinIdx == 0) && ((mode == 3) || (mode == 4))) {
		for(size_t i = 0; i < integralWindow; i++)
			spinErrors.push_back(0.0);
	}

	// Set reference based on the mode
	Vector6d refState;
	if(mode == 1 || mode == 2) refState << 0.0, 0.0, state0(2), 0.0, state0(4), 0.0;
	else if(mode == 3) refState << 0.0, 0.0, state0(2), 0.0, state0(4) + offsetAngle, 0.0;
	else if(mode == 4) refState << 0.0, 0.0, state0(2), 0.0, state0(4) - offsetAngle, 0.0;
	else if(mode == 5) {
		ul = ur = 15.0;
		return;
	}
	else {
		ul = ur = 0.0;
		return;
	}
	if(dbg) DISPLAY_VECTOR(refState);

	// Reset the integral based on the mode
	if((mode != 3) && (mode != 4)) {
		spinErrors.clear();
		spinIdx = 0;
	}

	// Set the gains
	Vector6d K;
	if(mode == 1) K = K_stand;
	else if(mode == 2) K = K_bal;
	else if(mode == 3) K = K_turn_left;
	else if(mode == 4) K = K_turn_right;
	else assert(false);

	// Compute the error
	Vector6d error = state - refState;
	if(dbg) DISPLAY_VECTOR(error);

	// Compute the torques 
	double u_theta = K.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());
	double u_x = K(2)*error(2) + K(3)*error(3);
	double u_spin = K.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());

	// Add integral
	if(((mode == 3) || (mode == 4)) && (error(4) < intErrorLimit)) {
		
		// Update the errors
		spinErrors[spinIdx % integralWindow] = error(4);
		spinIdx++;

		// Compute the total error
		double totalError = 0.0;
		for(size_t i = 0; i < integralWindow; i++)
			totalError += spinErrors[i];

		// Compute the addition
		double u_spin_int = totalError * Ksint;
		if(dbg) printf("u_spin_int: %lf\n", u_spin_int);
		u_spin += u_spin_int;
	}

	// Limit the output torques
	if(dbg) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);
	u_spin += extraSpin;
	u_spin = max(-12.0, min(12.0, u_spin));
	ul = u_theta + u_x + u_spin;
	ur = u_theta + u_x - u_spin;
	ul = max(-50.0, min(50.0, ul));
	ur = max(-50.0, min(50.0, ur));
}

/* ******************************************************************************************** */
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing data until stop received
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	Vector6d state;
	int lastMode = mode;
	while(!somatic_sig_received) {

		pthread_mutex_lock(&mutex);
		dbg = (c_++ % 30 == 0);
		if(dbg) cout << "\nmode: " << mode << endl;
		if(dbg) cout << "extra spin: " << extraSpin << endl;
		if(dbg) cout << "offset angle: " << offsetAngle << ", deg: " <<
			offsetAngle * (180.0 / M_PI) << endl;

		// Read the gains if requested by user
		if(shouldRead) {
			readGains();
			shouldRead = false;
		}

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Get the state 
		getState(state, dt); 
		if(dbg) DISPLAY_VECTOR(state);

		// Update state0 if the mode has been changed by user from 3-4 to 2
		if((lastMode == 3 || lastMode == 4) && (mode == 2)) state0 = state;
		if((lastMode == 0) && (mode == 1)) state0 = state;
//		if((lastMode == 2) && (mode == 3)) state0 = state;
//		if((lastMode == 2) && (mode == 4)) state0 = state;
		if(dbg) DISPLAY_VECTOR(state0);

		// Switch the mode if necessary
		switchModes(state);

		// Compute the torques based on the state and the desired torque
		double ul, ur;
		computeTorques(state, ul, ur);

		// Apply the torque
		double input [2] = {ul, ur};
		if(dbg) cout << "u: {" << ul << ", " << ur << "}" << endl;
		if(start) somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
		lastMode = mode;
		pthread_mutex_unlock(&mutex);
	}

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// Read the gains
	readGains();

	// Load the world and the robot
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton(0);

	// Initialize the daemon and the drivers
	init();
	
	// Print the f/t values
	run();

	// Destroy the daemon and the robot
	somatic_d_destroy(&daemon_cx);
	delete krang;
	return 0;
}
