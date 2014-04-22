/**
 * @file 04-trajectory.cpp
 * @author Can Erdogan, Jon Scholz
 * @date April 09, 2014
 * @brief Implements PID control to follow a trajectory. The global state of the robot is 
 * kept by combining vision data and odometry. If vision data is received, odometry is reset 
 * to stop error built-ups. 
 * The state is the 6x1 (x,x.,y,y.,th,th.) of the robot where (x,y,th) is the transform that
 * represents the robot in the global frame. 
 */

#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>

#include <kore.hpp>

#define SQ(x) ((x) * (x))
#define R2D(x) (((x) / M_PI) * 180.0)

Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) { return mat; }
using namespace std;

/* ******************************************************************************************** */
ach_channel_t state_chan, vision_chan, base_waypts_chan;
somatic_d_t daemon_cx;				//< The context of the current daemon
Krang::Hardware* krang;				//< Interface for the motor and sensors on the hardware
simulation::World* world;			//< the world representation in dart
dynamics::SkeletonDynamics* robot;			//< the robot representation in dart

/* ******************************************************************************************** */
bool start = false;
bool dbg = false;
bool shouldRead = false;
size_t mode = 0;		//< 0 sitting, 1 move on ground following keyboard commands
bool jumpPermission = true;

/* ******************************************************************************************** */
typedef Eigen::Matrix<double,6,1> Vector6d;
Vector6d refState;
Vector6d state;					//< current state (x,x.,y,y.,th,th.)
Eigen::Vector4d wheelsState;		 //< wheel pos and vels in radians (lphi, lphi., rphi, rphi.)
Eigen::Vector4d lastWheelsState; //< last wheel state used to update the state 
vector <Vector6d> trajectory;		//< the goal trajectory	
size_t trajIdx = 0;
FILE* file;							//< used to print the state when the next trajectory index is used

/* ******************************************************************************************** */
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;	//< mutex to update gains
Eigen::Vector4d K;	//< the gains for x and th of the state. y is ignored.

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {
	ifstream file ("/home/cerdogan/Documents/Software/project/krang/experiments/navigation/data/gains-04.txt");
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
		else if(input==' ') jumpPermission = !jumpPermission;
		pthread_mutex_unlock(&mutex);
	}
}

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void updateWheelsState(Eigen::Vector4d& wheelsState, double dt) {

	// Model constants
	static const double width = 0.69; //< krang base width in meters (measured: 0.69)
	static const double wheel_diameter = 0.536;
	static const double k1 = 0.475;//< scaling constant to make angle actually work. old: 0.55
	static const double k2 = 0.503; //< scaling constant to make displacement actually work: 5

	// Update the sensor information
	krang->updateSensors(dt);

	// Change the wheel values from radians to meters
	double tleft = krang->amc->pos[0] * wheel_diameter;  //< traversed distances for left wheel in meters
	double tright = krang->amc->pos[1] * wheel_diameter; //< traversed distances for right wheel in meters
	double vleft = krang->amc->vel[0] * wheel_diameter;  //< left wheel velocity in m/s
	double vright = krang->amc->vel[1] * wheel_diameter; //< right wheel velocity in m/s

	// Set the state
	wheelsState(0) = k2*(tleft + tright)/2.0;// + krang->imu;
	wheelsState(1) = k2*(vleft + vright)/2.0;// + krang->imuSpeed;
	wheelsState(2) = k1*(tright - tleft)/width; // (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	wheelsState(3) = k1*(vright - vleft)/width; // TODO: verify
}

/* ******************************************************************************************** */
/// Update the state using the wheels information with Euler approximation (we use the last
/// theta with the forward motion to reflect changes on x and y axes)
void updateState(Eigen::Vector4d& wheelsState, Eigen::Vector4d& lastWheelsState, Vector6d& state) {

	// Compute the change in forward direction and orientation
	double dx = wheelsState[0] - lastWheelsState[0];
	double dt = wheelsState[2] - lastWheelsState[2];
	double last_theta = state[4]; 

	state[0] = state[0] + dx*cos(last_theta);
	state[1] = wheelsState[1];
	state[2] = state[2] + dx*sin(last_theta);
	state[3] = 0;
	state[4] = state[4] + dt;
	state[5] = wheelsState[3];
}

/* ******************************************************************************************** */
void init() {

	// Open the file
	file = fopen("bla", "w+");
	assert((file != NULL) && "Could not open the file");

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "04-trajectory";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot); 

	// Open channel to publish the state
	enum ach_status r = ach_open(&state_chan, "krang_global", NULL);
	assert(ACH_OK == r);
	r = ach_flush(&state_chan);

	// Open channel to receive vision data
	r = ach_open(&vision_chan, "krang_vision", NULL );
	assert(ACH_OK == r);
	r = ach_flush(&vision_chan);

	// Open channel to receive waypoints data
	r = ach_open(&base_waypts_chan, "krang_base_waypts", NULL );
	assert(ACH_OK == r);
	r = ach_flush(&base_waypts_chan);

	// Receive the current state from vision
	double rtraj[1][3] = {1, 2, 3};
	size_t frame_size = 0;
	cout << "waiting for initial vision data: " << endl;
	r = ach_get(&vision_chan, &rtraj, sizeof(rtraj), &frame_size, NULL, ACH_O_WAIT);
	assert(ACH_OK == r);
	for(size_t i = 0; i < 3; i++) printf("%lf\t", rtraj[0][i]);
	printf("\n");

	// Set the state, refstate and limits
	updateWheelsState(wheelsState, 0.0);
	lastWheelsState = wheelsState;
	updateState(wheelsState, lastWheelsState, state);

	// Reset the current state to vision data
	for(size_t i = 0; i < 3; i++) state(2*i) = rtraj[0][i];
	state(1) = state(3) = state(5) = 0.0;
	cout << "state: " << state.transpose() << ", OK?" << endl;
	// getchar();
	refState = state;

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

}

/* ******************************************************************************************** */
void computeTorques (const Vector6d& state, double& ul, double& ur) {

	static double lastUx = 0.0;

	// Set reference based on the mode
	if(mode == 0) {
		ul = ur = 0.0;
		return;
	}

	// Compute the linear pos error by projecting the reference state's position in the current
	// state frame to the current heading direction
	Eigen::Vector2d dir (cos(state(4)), sin(state(4)));
	Eigen::Vector2d refInCurr (refState(0) - state(0), refState(2) - state(2));
	double linear_pos_err = dir.dot(refInCurr);
	double linear_vel_err = state(1);
	
	// Compute the angular position error by taking the difference of the two headings
	double angular_pos_err = refState(4) - state(4);
	double angular_vel_err = state(5);

	// Compute the error and set the y-components to 0
	Eigen::Vector4d error (linear_pos_err, linear_vel_err, angular_pos_err, angular_vel_err);
	if(dbg) cout << "error: " << error.transpose() << endl;
	if(dbg) cout << "K: " << K.transpose() << endl;

	// Compute the forward and spin torques (note K is 4x1 for x and th)
	double u_x = (K(0)*error(0) + K(1)*error(1));
	double u_spin = (K(2)*error(2) + K(3)*error(3));

	// Limit the output torques
	u_spin = max(-20.0, min(20.0, u_spin));
	u_x= max(-15.0, min(15.0, u_x));
	if(dbg) printf("u_x: %lf, u_spin: %lf\n", u_x, u_spin);
	ul = u_x - u_spin;
	ur = u_x + u_spin;
	ul = max(-30.0, min(30.0, ul));
	ur = max(-30.0, min(30.0, ur));
	if(dbg) printf("ul: %lf, ur: %lf\n", ul, ur);
}

/* ******************************************************************************************** */
/// Updates the trajectory using the waypoints channel
void updateTrajectory () {

	// Check if a message is received
	const size_t k = 170;
	double rtraj[k][3] = {0};
	size_t frame_size = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
	ach_status_t r = ach_get(&base_waypts_chan, &rtraj, sizeof(rtraj), &frame_size, 
			&abstimeout, ACH_O_LAST);
	if(!(r == ACH_OK || r == ACH_MISSED_FRAME)) return;

	// Fill the trajectory data (3 doubles, 8 bytes, for each traj. point)
	trajectory.clear();
	size_t numPoints = frame_size / (8 * 3);
	for(size_t p_idx = 0; p_idx < numPoints; p_idx++) {
		Vector6d refState;
		refState << rtraj[p_idx][0], 0.0, rtraj[p_idx][1], 0.0, rtraj[p_idx][2], 0.0;
		trajectory.push_back(refState);
	}

	// Update the reference state
	trajIdx = 0;
	refState = trajectory[0];
}

/* ******************************************************************************************** */
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing data until stop received
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	int lastMode = mode;
	struct timespec t_forwStart;
	while(!somatic_sig_received) {

		pthread_mutex_lock(&mutex);
		dbg = (c_++ % 20 == 0);
		if(dbg) cout << "\nmode: " << mode;
		if(dbg) cout << " start: " << start;
		if(dbg) cout << " jumpPerm: " << jumpPermission << endl;
	
		// Read the gains if requested by user
		if(shouldRead) {
			readGains();
			shouldRead = false;
		}

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Get the state and update odometry
		lastWheelsState = wheelsState;
		updateWheelsState(wheelsState, dt); 
		if(dbg) cout << "wheelsState: " << wheelsState.transpose() << endl;
		updateState(wheelsState, lastWheelsState, state);
		if(dbg) {
			fprintf(file, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t0\n", state(0), state(2), state(4), 
				refState(0), refState(2), refState(4));
			fflush(file);
		}
		if(dbg) cout << "state: " << state.transpose() << endl;

		// Check if a new trajectory data is given
		updateTrajectory();
		
		// Update the reference state if necessary
		if(!trajectory.empty()) {

			Eigen::Vector2d dir (cos(state(4)), sin(state(4)));
			Eigen::Vector2d refInCurr (refState(0) - state(0), refState(2) - state(2));
			double xerror = dir.dot(refInCurr);
			double therror = SQ(refState(4) - state(4));
			static const double xErrorThres = -0.001;
			static const double thErrorThres = SQ(0.06);
			// if(dbg) cout << "traj idx xerror: " << (sqrt(xerror)) << ", vs. " << (sqrt(xErrorThres)) << endl;
			if(dbg) cout << "traj idx xerror: " << (xerror) << ", vs. " << xErrorThres  << endl;
			if(dbg) cout << "traj idx therror (deg): " << R2D(sqrt(therror)) << ", vs. " << R2D(sqrt(thErrorThres)) << endl;
			if(dbg) cout << "jump permission: " << jumpPermission << endl;
			bool reached = (xerror < xErrorThres) && (therror < thErrorThres);

			if(dbg) printf("reached: %d, xreached: %d, threached: %d\n", reached,
				(xerror < xErrorThres), (therror < thErrorThres));
			if(jumpPermission & reached) {
				fprintf(file, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t1\n", state(0), state(2), state(4), 
					refState(0), refState(2), refState(4));
				fflush(file);
				trajIdx = min((trajIdx + 1), (trajectory.size() - 1));
				refState = trajectory[trajIdx];
//				jumpPermission = false;
				if(trajIdx == trajectory.size() - 1) mode = 0;
			}
		}
		if(dbg) cout << "traj idx: " << trajIdx << ", traj size: " << trajectory.size() << endl;
		if(dbg) cout << "refState: " << refState.transpose() << endl;

		// Send the state
		if(true) {
		  double traj[1][6] = {{state(0), state(1), state(2), state(3), state(4), state(5)}};
		  //if(dbg) cout << "sending state: " << state.transpose() << endl;
		  ach_put(&state_chan, &traj, sizeof(traj));
		}

		// Compute the torques based on the state and the mode
		double ul, ur;
		computeTorques(state, ul, ur);

		// Apply the torque
		double input [2] = {ul, ur};
		if(dbg) cout << "u: {" << ul << ", " << ur << "}" << endl;
		if(!start) input[0] = input[1] = 0.0;
		somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
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
