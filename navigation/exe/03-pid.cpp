/**
 * @file 03-pid.cpp
 * @author Can Erdogan
 * @date April 01, 2014
 * @brief Implements PID control to control the forward and spin of the robot while sitting on the
 * ground.
 */

#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <initModules.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>

#include <kore.hpp>

Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) { return mat; }

using namespace std;

somatic_d_t daemon_cx;				///< The context of the current daemon
Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
simulation::World* world;			///< the world representation in dart
dynamics::SkeletonDynamics* robot;			///< the robot representation in dart

bool start = false;
bool dbg = false;
bool shouldRead = false;
size_t integralWindow = 0;
double Kfint, intErrorLimit;

Eigen::Vector4d K;
Eigen::Vector4d refState;
Eigen::Vector4d state;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
size_t mode = 0;		// 0 sitting, 1 move on ground following keyboard commands

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {
	ifstream file ("/home/cerdogan/Documents/Software/project/krang/experiments/navigation/data/gains-03.txt");
	assert(file.is_open());
	char line [1024];
	K = Eigen::Vector4d::Zero();
	file.getline(line, 1024);
	std::stringstream stream(line, std::stringstream::in);
	size_t i = 0;
	double newDouble;
	while ((i < 4) && (stream >> newDouble)) K(i++) = newDouble;
}

/* ********************************************************************************************* */
/// Get the mode input
void *kbhit(void *) {
	char input;
	double kOffset = 0.05;
	while(true){ 
		input=cin.get(); 
		pthread_mutex_lock(&mutex);
		if(input=='0') mode = 0;
		else if(input=='1') mode = 1;
		else if(input=='d') dbg = !dbg;
		else if(input=='s') start = !start;
		else if(input=='r') shouldRead = true;
		else if(input=='f') refState = state;
		else if(input=='i') refState(0) += kOffset;
		else if(input=='k') refState(0) -= kOffset;
		else if(input=='j') refState(2) += kOffset;
		else if(input=='l') refState(2) -= kOffset;
		pthread_mutex_unlock(&mutex);
	}
}

/* ******************************************************************************************** */
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "03-pid";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot); 

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Eigen::Vector4d& state, double dt) {
  krang->updateSensors(dt);
	state(0) = (krang->amc->pos[0] + krang->amc->pos[1])/2.0 + krang->imu;
	state(1) = (krang->amc->vel[0] + krang->amc->vel[1])/2.0 + krang->imuSpeed;
	state(2) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	state(3) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;
}

/* ******************************************************************************************** */
void computeTorques (const Eigen::Vector4d& state, double& ul, double& ur) {

	static double lastUx = 0.0;

	// Set reference based on the mode
	if(mode == 0) {
		ul = ur = 0.0;
		return;
	}
	if(dbg) cout << "refState: " << refState.transpose() << endl;

	// Compute the error
	Eigen::Vector4d error = state - refState;
	if(dbg) cout << "error: " << error.transpose() << endl;
	if(dbg) cout << "K: " << K.transpose() << endl;

	// Compute the forward and spin torques 
	double u_x = K(0)*error(0) + K(1)*error(1);
	double u_spin = K.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());

	// Limit the output torques
	if(dbg) printf("u_x: %lf, u_spin: %lf\n", u_x, u_spin);
	u_spin = max(-10.0, min(10.0, u_spin));
	ul = u_x + u_spin;
	ur = u_x - u_spin;
	ul = max(-50.0, min(50.0, ul));
	ur = max(-50.0, min(50.0, ur));
	if(dbg) printf("ul: %lf, ur: %lf\n", ul, ur);
}

/* ******************************************************************************************** */
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing data until stop received
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	Eigen::Vector4d state;
	int lastMode = mode;
	struct timespec t_forwStart;
	while(!somatic_sig_received) {

		pthread_mutex_lock(&mutex);
		dbg = (c_++ % 20 == 0);
		if(dbg) cout << "\nmode: " << mode << endl;
		
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
		if(dbg) cout << "state: " << state.transpose() << endl;

		// Compute the torques based on the state and the mode
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
	getState(state, 0.0);
	refState = state;
	
	// Print the f/t values
	run();

	// Destroy the daemon and the robot
	somatic_d_destroy(&daemon_cx);
	delete krang;
	return 0;
}
