/**
 * @file 02-forward.cpp
 * @author Can Erdogan
 * @date Feb 14, 2014
 * @brief Demonstrates the moving forward/backward controller.
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

typedef Eigen::Matrix<double,6,1> Vector6d;

using namespace std;

somatic_d_t daemon_cx;				///< The context of the current daemon
Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
simulation::World* world;			///< the world representation in dart
dynamics::SkeletonDynamics* robot;			///< the robot representation in dart

Vector6d K_stand, K_bal, K_forw;
Vector6d state0;	///< in the beginning of entering mode 2
bool start = false;
bool dbg = false;
bool shouldRead = false;
double extraForw = 0.0;
size_t integralWindow = 0;
double Kfint, intErrorLimit;
double offsetDist = 0.3;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
size_t mode = 0;		// 0 sitting, 1 standing up, 2 stable, 3 turning, 4 sitting down

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	// Get the gains
	Vector6d* kgains [] = {&K_stand, &K_bal, &K_forw};
	ifstream file ("/home/cerdogan/Documents/Software/project/krang/experiments/navigation/data/gains-02.txt");
	assert(file.is_open());
	char line [1024];
	for(size_t k_idx = 0; k_idx < 3; k_idx++) {
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
	stream2 >> Kfint;
	file.getline(line, 1024);
	std::stringstream stream3(line, std::stringstream::in);
	stream3 >> intErrorLimit;
	file.close();
	printf("READ GAINS!\n");
	pv(K_forw);
	printf("integralWindow: %d, Kfint: %lf, intErrorLimit: %lf\n", 
		integralWindow, Kfint, intErrorLimit);
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
		else if(input=='k') extraForw += 1.0;
		else if(input=='j') extraForw += -1.0;
		else if(input=='s') start = !start;
		else if(input=='r') shouldRead = true;
		else if(input=='h') offsetDist += 0.1;
		else if(input=='l') offsetDist -= 0.1;
		pthread_mutex_unlock(&mutex);
	}
}

/* ******************************************************************************************** */
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "02-forward";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot); 

	// Check that the waist is at the expected angle
	// assert(fabs(krang->waist->pos[0] - 2.86) < 0.05 && "The gains and offsets are set for 164 deg");
	
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
	com(0) += 0.0052;  // for 164 waist
	com(0) += 0.0094;  // for weird left arm and 156 waist
	com(0) += 0.0051;  // for cinder weight
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
		if(fabs(state(0)) < 0.064) balancedCounter++;
		if(balancedCounter > 100) {
			mode = 2;
			balancedCounter = 0;
			state0 = state;
		}	
	}
}

/* ******************************************************************************************** */
void computeTorques (const Vector6d& state, double& ul, double& ur) {

	static double lastUx = 0.0;

	// Set reference based on the mode
	Vector6d refState;
	if(mode == 1 || mode == 2) refState << 0.0, 0.0, state0(2), 0.0, state0(4), 0.0;
	else if(mode == 5) {
		ul = ur = 15.0;
		return;
	}
	else {
		ul = ur = 0.0;
		return;
	}
	if(dbg) cout << "refState: " << refState.transpose() << endl;

	// Set the gains
	Vector6d K;
	if(mode == 1) K = K_stand;
	else if(mode == 2) K = K_bal;
	else assert(false);

	// Compute the error
	Vector6d error = state - refState;
	if(dbg) cout << "error: " << error.transpose() << endl;

	// Compute the forward and spin torques 
	double u_x = K(2)*error(2) + K(3)*error(3);
	double u_spin = K.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());
	double u_theta = K.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());

	// Limit the output torques
	if(dbg) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);
	u_spin = max(-10.0, min(10.0, u_spin));
	ul = u_theta + u_x + u_spin;
	ur = u_theta + u_x - u_spin;
	ul = max(-50.0, min(50.0, ul));
	ur = max(-50.0, min(50.0, ur));
	if(dbg) printf("ul: %lf, ur: %lf\n", ul, ur);
}

/* ******************************************************************************************** */
void forwardTorques (const Vector6d& state, double time, double dt, double& ul, double& ur) {

	bool localDbg = 0;

	// Define the profile
	static const double acceleration = 0.008;		// m/s
	static const double accelerationTime = 15.0;	// seconds
	static const double deceleration = 0.008;		// m/s
	static const double decelerationTime = 15.0;	// seconds
	static const double cruiseTime = 30.0;

//	cout << "state: " << state.transpose() << endl;

	// Get the current reference velocity
	double refVel = 0.0;
	if(time < accelerationTime) refVel = time * acceleration;
	else if(time < (accelerationTime + cruiseTime)) refVel = accelerationTime * acceleration;
	else if(time < (accelerationTime + cruiseTime + decelerationTime))
		refVel = accelerationTime * acceleration - (time - accelerationTime - cruiseTime) * deceleration;
	else refVel = 0.0;
	
	// Get the position from the reference velocities
	double refPos = 0.0;
	if(time < accelerationTime) refPos = (time * (time * acceleration)) / 2.0;
	else if(time < (accelerationTime + cruiseTime))
		refPos = (accelerationTime * (accelerationTime * acceleration)) / 2.0 + 
							(time - accelerationTime) * (accelerationTime * acceleration);
	else if(time < (accelerationTime + cruiseTime + decelerationTime))
		refPos = (accelerationTime * (accelerationTime * acceleration)) / 2.0 + 
						 (cruiseTime * (accelerationTime * acceleration)) + 
						 (refVel * (time - accelerationTime - cruiseTime)) + 
						 (time - accelerationTime - cruiseTime) * (acceleration * accelerationTime - refVel) / 2.0;
	else refPos = (accelerationTime * (acceleration * accelerationTime)) / 2.0 + 
							  (cruiseTime * (accelerationTime * acceleration)) + 
								(decelerationTime * (deceleration * decelerationTime)) / 2.0;
	refPos += state0(2);

	// Determine acceleration
	static vector <double> accs;
	static int c_ = 0;
	c_++;
	if(accs.empty()) for(size_t i = 0; i < 150; i++) accs.push_back(0.0);
	static double lastVel = state(3);
	double lastAcc = (state(3) - lastVel) / dt;
	accs[c_ % 150] = lastAcc;
	double stateAcc = 0.0;
	for(size_t i = 0; i < 150; i++) stateAcc += accs[i];
	stateAcc /= 150;
	double refAcc = (time > accelerationTime) ? -deceleration : acceleration;
	lastVel = state(3);
	printf("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", time, refPos, refVel, refAcc, state(2), state(3), stateAcc);

	// Create the reference state
	Vector6d refState;
	refState << 0.0, 0.0, refPos, refVel, state(4), 0.0;
	if(localDbg) cout << "refState: " << refState.transpose() << endl;

	// Set the torques based on the current state and the reference velocity and position values
	double errPos = refPos - state(2), errVel = refVel - state(3), errAcc = refAcc - stateAcc;
	double u_x = K_forw(2) * errPos + K_forw(3) * errVel + Kfint * errAcc;

	// Extra stuff
	u_x += extraForw;
	if(localDbg) printf("u_x before cap: %lf\n", u_x);
	u_x = max(-10.0, min(10.0, u_x));

	// Update the reference balancing angle
	static const double totalMass = 140.00;
	Eigen::Vector3d com = robot->getWorldCOM();
	double com_x = -(u_x / 1.7) / (totalMass * 9.81);
	double normSq = com(0) * com(0) + com(2) * com(2);
	double com_z = sqrt(normSq - com_x * com_x);
	refState(0) = atan2(-com_x, com_z);
//	refState(0) = 0.0;
	
	// Compute the error
	Vector6d error = state - refState;
	if(localDbg) cout << "error: " << error.transpose() << endl;

	// Compute the balancing and spin inputs
	double u_spin = K_forw.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());
	double u_theta = K_forw.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());

	// Limit the output torques
	if(localDbg) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);
	u_spin = max(-10.0, min(10.0, u_spin));
	ul = u_theta - u_x + u_spin;
	ur = u_theta - u_x - u_spin;
	ul = max(-50.0, min(50.0, ul));
	ur = max(-50.0, min(50.0, ur));
	ul *= intErrorLimit;
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
	struct timespec t_forwStart;
	while(!somatic_sig_received) {

		pthread_mutex_lock(&mutex);
		if(mode == 3) dbg = false;
		else dbg = (c_++ % 20 == 0);
		if(dbg) cout << "\nmode: " << mode << endl;
		if(dbg) cout << "extra forw: " << extraForw << endl;
		if(dbg) cout << "offset dist: " << offsetDist << endl;

		// Keep track of the starting time to follow the velocity profile
		if((lastMode != 3) && (mode == 3)) {
			t_forwStart = aa_tm_now();
		}
		
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

		// Update state0 if the mode has been changed by user from 3-4 to 2
		if((lastMode == 3 || lastMode == 4) && (mode == 2)) state0 = state;
		if(dbg) cout << "state0: " << state0.transpose() << endl;

		// Switch the mode if necessary
		switchModes(state);

		// Compute the torques based on the state and the mode
		double ul, ur;
		if(mode != 3 && mode != 4) computeTorques(state, ul, ur);
		else {
			double profileTime = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_forwStart));
			forwardTorques(state, profileTime, dt, ul, ur);
		}

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
