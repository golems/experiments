/**
 * @file basenavd.cpp
 * @author Jon Scholz, Can Erdogan
 * @date March 02, 2015
 * @brief Implements PID control to follow a trajectory. The global state of the robot is 
 * kept by combining vision data and odometry. If vision data is received, odometry is reset 
 * to stop error built-ups. 
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
#include <kore/display.hpp>
#include <kore/util.hpp>

#include <trajectory_io.h>
#include <argp.h>
#include <ncurses.h>

#define SQ(x) ((x) * (x))
#define R2D(x) (((x) / M_PI) * 180.0)

#define MODE_OFF 1
#define MODE_KBD 2
#define MODE_TRAJ 3

/* Prints a row or column vector. */
#define PRINT_VECTOR(X)                                   \
			printw("(%d x %d) ", (X).rows(), (X).cols()); \
			printw("[");                                  \
			for(int i = 0; i < (X).size(); i++){          \
				printw(" %4.4f", (X)[i]);                 \
			}                                             \
			printw("]\n\r");


#define PRINT_MAT(M)                                     \
			printw("[");                                 \
			for(int i = 0; i<(M).rows(); ++i){           \
				printw(" [");                            \
				for(int j = 0; j < (M).cols(); j++)      \
					printw(" %4.4f", (M)(i,j));          \
				printw("]\n\r");                         \
			}                                            \
			printw("]\n");

using namespace std;

/* ---------- */
/* ARGP Info  */
/* ---------- */

// argp struct
typedef struct {
	somatic_d_t d;
	somatic_d_opts_t d_opts;
	const char *opt_base_state_chan_name;
	const char *opt_base_pose_chan_name;
	const char *opt_base_waypts_chan_name;
	int opt_verbosity;
} cx_t;

static struct argp_option options[] = {
	{
		.name = "verbose",
		.key = 'v',
		.arg = NULL,
		.flags = 0,
		.doc = "Causes verbose output"
	},
	{
		.name = "state_channel",
		.key = 's',
		.arg = "state_channel",
		.flags = 0,
		.doc = "ach channel to use for publishing krang base state"
	},
	{
		.name = "pose_channel",
		.key = 'p',
		.arg = "pose_channel",
		.flags = 0,
		.doc = "ach channel to use for reading krang base pose (from vision)"
	},
	{
		.name = "waypts_channel",
		.key = 'w',
		.arg = "waypts_channel",
		.flags = 0,
		.doc = "ach channel to use for reading krang base waypoint trajectories"
	},
	SOMATIC_D_ARGP_OPTS,
	{
		.name = NULL,
		.key = 0,
		.arg = NULL,
		.flags = 0,
		.doc = NULL
	}
};


/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "-0.0.1";
/// argp program arguments documentation
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "Runs the krang base navigation controller";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


static int parse_opt( int key, char *arg, struct argp_state *state) {
	cx_t *cx = (cx_t*)state->input;
	switch(key) {
	case 'v':
		cx->opt_verbosity++;
		break;
	case 's':
		cx->opt_base_state_chan_name = strdup( arg );
		break;
	case 'p':
		cx->opt_base_pose_chan_name = strdup( arg );
		break;
	case 'w':
		cx->opt_base_waypts_chan_name = strdup( arg );
		break;
	case 0:
		break;
	}
	
	somatic_d_argp_parse( key, arg, &cx->d_opts );

	return 0;
}

/* ******************************************************************************************** */
ach_channel_t state_chan, vision_chan, base_waypts_chan;
cx_t cx;							//< the arg parse struct
Krang::Hardware* krang;				//< Interface for the motor and sensors on the hardware
simulation::World* world;			//< the world representation in dart
dynamics::SkeletonDynamics* robot;			//< the robot representation in dart

/* ******************************************************************************************** */
bool start = false;
bool dbg = false;
bool update_gains = false; 	//< if true, read new gains from the text file
size_t mode = 1;			//< 0 sitting, 1 keyboard commands, 2 trajectory following
bool advance_waypts = true;
bool wait_for_global_vision_msg_at_startup = false;
bool use_steering_method = false;
const double waypt_lin_thresh = 0.01; // 0.003; // 0.001;
const double waypt_rot_thresh = 0.015; // 0.0036;
bool error_integration = false;
double ang_int_err;					//< accumulator for angular error
double lin_int_err;					//< accumulator for linear error (x in robot frame)
double u_hard_max = 18.0;			//< never write values higher than this
double u_spin_max = 18.0; 			//< thershold on spin contribution
double u_lin_max = 15.0; 			//< threshold on linear contribution
// bool save_hist = false;				//< save tracking history flag

/* ******************************************************************************************** */
typedef Eigen::Matrix<double,6,1> Vector6d;
Vector6d refstate;					//< reference state for controller (x,y,th,x.,y.,th.)
Vector6d state;						//< current state (x,y,th,x.,y.,th.)
Eigen::Vector4d wheel_state;		//< wheel pos and vels in radians (lphi, lphi., rphi, rphi.)
Eigen::Vector4d last_wheel_state; 	//< last wheel state used to update the state 
vector <Vector6d> trajectory;		//< the goal trajectory	
const size_t max_traj_msg_len = 170;
size_t trajIdx = 0;
// FILE* state_hist_file;				//< used to dump state and reference poses over time to plot traj. tracking

/* ******************************************************************************************** */
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;	//< mutex to update gains
Eigen::VectorXd K = Eigen::VectorXd::Zero(7);	//< the gains for x and th of the state. y is ignored.

// display information
const int CURSES_DEBUG_DISPLAY_START = 20;

// // Enum for control modes
// typedef enum Modes {
// 	DEFAULT = 0,
// 	OFF = 1,
// 	KBD = 2,
// 	TRAJ = 3
// } Modes;

/* ******************************************************************************************** */

/*
 * Returns the angle expressed in radians between -Pi and Pi
 */ 
double unwrap(double angle) 
{
	return fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

/// Read file for gains
void readGains () {
	ifstream file ("/home/jscholz/vc/experiments/navigation/data/gains-PID.txt");
	// ifstream file ("../data/gains-PID.txt");
	assert(file.is_open());
	char line [1024];
	// K = Eigen::VectorXd::Zero(7);
	file.getline(line, 1024);
	while (line[0] == '#')
		file.getline(line, 1024);
	std::stringstream stream(line, std::stringstream::in);
	size_t i = 0;
	double newDouble;
	while ((i < 7) && (stream >> newDouble)) K(i++) = newDouble;
	file.close();
}

/*
 * Updates the pose params of the robots state from vision messages

 * @param block: if true, wait for a message to come in before returning
 * @param reset_reference: if true, set the reference to the new state if 
 * 		a message is received.  This is a safety for keeping the robot from
 * 		lurching when a big state update comes in.  Should only be false
 * 		if polling to fix small errors while executing a trajectory.  
 */
void poll_vision_channel(bool block, bool reset_reference=true)
{
	// Receive the current state from vision
	// double rtraj[1][3] = {0, 0, 0};
	int bufsize = 2 * sizeof(int) + 3 * sizeof(double); // krang pose message size
	char buf[bufsize];

	size_t frame_size = 0;
	int options = block ? ACH_O_WAIT : ACH_O_LAST;
	int r = ach_get(&vision_chan, &buf, bufsize, &frame_size, NULL, options); 
	if(!(r == ACH_OK || r == ACH_MISSED_FRAME)) 
		return;

	// Set the current state to vision data
	Eigen::MatrixXd M = deserialize_to_Matrix(buf);
	state.head(3) = M.row(0).head(3); // M.topLeftCorner(1,3);
	state(3) = state(4) = state(5) = 0.0; // set vels to zero (will be over-written by odometry)
	// cout << "Updated state from vision message: " << state.transpose() << endl;
	printw("Updated state from vision message: \n"); PRINT_VECTOR(state.transpose())

	// reset odometry
	last_wheel_state = wheel_state;

	// never change the state without resetting the reference!
	if (reset_reference)
		refstate = state;
}

/*
 * A single setter for refstate, so we have the chance to use as
 * trigger for other events (flags and stuff)
 */
void setReference(Vector6d& newRef) {
	refstate = newRef;
}

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
/*
 * wheel_state is a 4-vector containing:
 * linear position (total distance traveled since start; average of both wheels)
 * linear velocity (current linear vel; average of both wheels)
 * angular position (difference of wheel dist over wheelbase)
 * angular velocity (difference of wheel vel over wheelbase)
 */
void updateWheelsState(Eigen::Vector4d& wheel_state, double dt) {

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
	double vleft = krang->amc->vel[0] * wheel_diameter;  //< left wheel velocity in m/s <-- cafeful: noisy!
	double vright = krang->amc->vel[1] * wheel_diameter; //< right wheel velocity in m/s <-- cafeful: noisy!

	// Set the state
	wheel_state[0] = k2 * (tleft + tright)/2.0; // + krang->imu;
	wheel_state[1] = (wheel_state[0] - last_wheel_state[0]) * dt; // finite diff. rather than amc vels b/c of noise
	wheel_state[2] = k1 * (tright - tleft)/width; // (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	wheel_state[3] = (wheel_state[2] - last_wheel_state[2]) * dt; // finite diff. rather than amc vels b/c of noise
}

/* ******************************************************************************************** */

/* Update the state using the wheels information with Euler approximation (we use the last
 * theta with the forward motion to reflect changes on x and y axes)
 *
 * state is a 6-vector containing:
 * 0: x
 * 1: y
 * 2: theta
 * 3: xdot
 * 4: ydot
 * 5: thetadot
 */
void updateState(Eigen::Vector4d& wheel_state, Eigen::Vector4d& last_wheel_state, Vector6d& state) {

	// Compute the change in forward direction and orientation
	double dlin = wheel_state[0] - last_wheel_state[0]; // linear distance covered (in *some* direction)
	double dtheta = wheel_state[2] - last_wheel_state[2]; // angular distance covered
	double last_theta = state[2]; // last robot heading is whatever theta was before we update it

	state[0] += dlin * cos(last_theta);				//< x
	state[1] += dlin * sin(last_theta);				//< y
	state[2] += dtheta;								//< theta
	state[3] = wheel_state[1] * cos(last_theta); 	//< xdot
	state[4] = wheel_state[1] * sin(last_theta); 	//< ydot
	state[5] = wheel_state[3];						//< thetadot
}

/* 
 * Computes the error between the provided state and reference state vectors.
 * Both are assumed to be in the world (or odom) frame.
 */
Vector6d computeError(const Vector6d& state, const Vector6d& refstate, bool rotate=true)
{
	// Error vector in world (or odom) frame, which is where state and refstate are defined
	Vector6d err = refstate - state;
	err[2] = unwrap(err[2]); //< unwrap angular portion of error
	Eigen::Rotation2Dd rot(state[2]); // used for rotating errors between robot and world frame

	// strict steering method:
	if (use_steering_method) {
		double refpos_angle = atan2(err[1], err[0]); 					//< angle towards current waypoint
		double refpos_dist = err.segment(0,2).norm(); 					//< linear distance to current waypoint
		double refpos_angle_err = unwrap(refpos_angle - state[2]); 		//< heading error 
		double ref_xerr_robot = (rot.inverse() * err.segment(0,2))[0]; 	//< small negative x error in robot frame

		if (refpos_dist > waypt_lin_thresh * 3				//< if we're far from the waypoint
			&& ref_xerr_robot > -waypt_lin_thresh * 3) 		//< ignores minor overshooting
			err[2] = refpos_angle_err; 						//< only worry about turning towards it
		
		if (err[2] > waypt_rot_thresh * 3)
			err.segment(0,2).setZero(); 					//< zero all translation controls if we're rotating
	}

	// if requested, convert error vector to robot frame, which is where we do control
	if (rotate) {
		err.segment(0,2) = rot.inverse() * err.segment(0,2); //< rotate position error to robot frame
		err.segment(3,2) = rot.inverse() * err.segment(3,2); //< rotate velocity error to robot frame	
	}

	return err;
}

/* ******************************************************************************************** */
void computeTorques(const Vector6d& state, double& ul, double& ur, double& dt) {
	// Set reference based on the mode
	if(mode == 1 || mode == 0) {
		ul = ur = 0.0;
		return;
	}

	Vector6d err_robot = computeError(state, refstate, true);

	// Static-friction compensation: increase p-gain inversely with vel to get robot started
	double velsq = state.segment(3,2).squaredNorm();
	double p_booster = K[6] * exp(-20 * velsq); //< exponentially falls off as robot gains speed

	if (error_integration) {
		lin_int_err += err_robot[0] * dt;		//< only integrate error in direction we can control
		ang_int_err += err_robot[2] * dt;	//< integrate angular error
	}

	// if (dbg) cout << "error: " << err_robot.transpose() << endl;
	// if (dbg) cout << "ang_int_err: " << ang_int_err << " lin_int_err: " << lin_int_err << endl;
	// if (dbg) cout << "Gains: " << K.transpose() << endl;
	if (dbg) {
		printw("Reference state error:\n\t");  PRINT_VECTOR(err_robot)
		printw("Integrated errors (ang, lin): %f, %f\n", ang_int_err, lin_int_err);
	}

	// Compute the forward and rotation torques
	// note: K is organized as [linearP linearI linearD angularP angularI angularD p_booster]
	// 		 err_robot is organized as [x, y, theta, xdot, ydot, thetadot]
	double u_x 		= (K[0] * err_robot[0] * p_booster + K[1] * lin_int_err + K[2] * err_robot[3]);
	double u_theta 	= (K[3] * err_robot[2] * p_booster + K[4] * ang_int_err + K[5] * err_robot[5]);

	// Limit the output torques
	u_theta = max(-u_spin_max, min(u_spin_max, u_theta));
	u_x= max(-u_lin_max, min(u_lin_max, u_x));
	if(dbg) printw("u_x: %lf, u_theta: %lf\n", u_x, u_theta);
	ul = u_x - u_theta;
	ur = u_x + u_theta;
	ul = max(-u_hard_max, min(u_hard_max, ul));
	ur = max(-u_hard_max, min(u_hard_max, ur));
	if(dbg) printw("ul: %lf, ur: %lf\n", ul, ur);
}

/* ******************************************************************************************** */
/// Updates the trajectory using the waypoints channel
void updateTrajectory () {

	// allocate a buffer big enough for the incoming trjajectory
	char buf[max_traj_msg_len * 3 * sizeof(double)];

	// Check if a message is received
	size_t frame_size = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
	ach_status_t r = ach_get(&base_waypts_chan, &buf, sizeof(buf), 
		&frame_size, &abstimeout, ACH_O_LAST);
	if(!(r == ACH_OK || r == ACH_MISSED_FRAME)) return;

	// Fill the trajectory data (3 doubles, 8 bytes, for each traj. point)
	// Update: now that we're unpacking to Eigen, could plug matrix in for 
	// traj and create refState (with vels) in run loop, but why bother?
	trajectory.clear();
	Eigen::MatrixXd poses = deserialize_to_Matrix(buf);
	for(size_t i = 0; i < poses.rows(); i++) {
		Vector6d newRefState;
		//newRefState << poses[i][0], poses[i][1], poses[i][2], 0.0, 0.0, 0.0;
		newRefState << poses(i,0), poses(i,1), poses(i,2), 0.0, 0.0, 0.0;
		trajectory.push_back(newRefState);
	}
	static int tctr = 0;
	// cout << "updating trajectory from poses" << poses << endl;
	if (dbg)
		printw("updating trajectory from poses:\n"); PRINT_MAT(poses)
	tctr++;

	// Update the reference state
	trajIdx = 0;
	setReference(trajectory[0]);

	// turn on trajectory following mode
	// mode = 3; // TODO prob. shouldn't change mode without permission
}


/* ******************************************************************************************** */

/* ********************************************************************************************* */

/* Prints the keyboard keys and \corresponding actions to the console */
void print_key_bindings(){
	const char *str =
	 "' ': toggle ON and OFF sending commands to motors                         \n\r"
	 "'q'    : Quit                                                             \n\r"
	 "'r'    : Reset Reference                                                  \n\r"
	 "' '    : Start/Stop                                                       \n\r"
	 "'d'    : Toggle debug output                                              \n\r"
	 "'a'    : Toggle waypoint advance mode                                     \n\r"
	 "'1'    : Set Mode: OFF                                                    \n\r"
	 "'2'    : Set Mode: KEYBOARD                                               \n\r"
	 "'3'    : Set Mode: TRAJECTORY                                             \n\r"
	 "'j'    : Keyboard control: turn left                                      \n\r"
	 "'l'    : Keyboard control: turn right                                     \n\r"
	 "'i'    : Keyboard control: drive forward                                  \n\r"
	 "'k'    : Keyboard control: drive backward                                 \n\r";

	printw("KEY BINDINGS\n\r%s\r", str);
	return;
}

/// Get the mode input
/*
Keybindings:
'1' - mode: 
*/
void *kbhit(void *) {
	// char input;
	char ch;
	
	while(true){ 
		// input=cin.get(); 
		ch = getch(); 
		pthread_mutex_lock(&mutex);
		switch (ch) {
			case 'q': somatic_sig_received = true; break;
			// case 'r': {
			// 	somatic_motor_reset(&cx.d, krang->amc);
			// 	break;
			// } 
			case '1': {
				mode = 1;
				break;
			}
			case '2': {
				mode = 2;
				break;
			}
			case '3': {
				mode = 3;
				break;
			}
			case 'd': {
				dbg = !dbg;
				break;
			}
			case 'a': {
				advance_waypts = !advance_waypts;
				break;
			}
			case ' ': {
				start = !start;
				break;
			}
			case 'g': {
				update_gains = true; 	//< if true, read new gains from the text file
				break;
			}
			case 'r': {
				setReference(state);
				trajectory.clear();
				ang_int_err = 0;
				lin_int_err = 0;
				break;
			}
			case 'i': {
				if (mode != MODE_KBD) break;
				// input is in the the robot frame, so we rotate to world for refstate
				refstate(0) += 10 * waypt_lin_thresh * cos(state[2]);
				refstate(1) += 10 * waypt_lin_thresh * sin(state[2]);
				break;
			}
			case 'k': {
				if (mode != MODE_KBD) break;
				// input is in the the robot frame, so we rotate to world for refstate
				refstate(0) -= 10 * waypt_lin_thresh * cos(state[2]);
				refstate(1) -= 10 * waypt_lin_thresh * sin(state[2]);
				break;
			}
			case 'j': {
				if (mode != MODE_KBD) break;
				refstate(2) += 10 * waypt_rot_thresh;
				break;
			}
			case 'l': {
				if (mode != MODE_KBD) break;
				refstate(2) -= 10 * waypt_rot_thresh;
				break;
			}
			case 'o': {
				error_integration = !error_integration;
				break;
			}
			default:
				break;
		}
		pthread_mutex_unlock(&mutex);

		// if(input=='0') mode = 0;
		// else if(input=='1') mode = 1;
		// else if(input=='2') mode = 2;
		// else if(input=='d') dbg = !dbg;
		// else if(input=='s') start = !start;
		// else if(input=='g') update_gains = true; 	//< if true, read new gains from the text file
		// else if(input=='r') { 						//< set reference to current state at next iteration
		// 	setReference(state);
		// 	trajectory.clear();
		// 	ang_int_err = 0;
		// 	lin_int_err = 0;
		// }
		// else if(input=='i') { 
		// 	// input is in the the robot frame, so we rotate to world for refstate
		// 	refstate(0) += 10*waypt_lin_thresh * cos(state[2]);
		// 	refstate(1) += 10*waypt_lin_thresh * sin(state[2]);
		// }
		// else if(input=='k') {
		// 	// input is in the the robot frame, so we rotate to world for refstate
		// 	refstate(0) -= 10*waypt_lin_thresh * cos(state[2]);
		// 	refstate(1) -= 10*waypt_lin_thresh * sin(state[2]);
		// }
		// else if(input=='j') refstate(2) += 10*waypt_rot_thresh;
		// else if(input=='l') refstate(2) -= 10*waypt_rot_thresh;
		// else if(input=='o') error_integration = !error_integration;
		// else if(input==' ') advance_waypts = !advance_waypts;
		// pthread_mutex_unlock(&mutex);
	}
}

/* Prints the current status of the machine */
void print_status(const Krang::Hardware* hw, const cx_t& cx)
{
	printw("STATUS\n");
	
	printw("Sending motor commands:\t\t");
    printw(start ? "ON\n" : "OFF\n");
	
	printw("Control mode:\t\t\t");
	switch (mode) {
		case 1: printw("OFF\n"); break;
		case 2: printw("KEYBOARD\n"); break;
		case 3: printw("TRAJECTORY-FOLLOWING\n"); break;
	}

	printw("Allow waypoint advance:\t\t");
	printw(advance_waypts ? "ON\n" : "OFF\n");

	printw("Debug output:\t\t\t");
	printw(dbg ? "ON\n" : "OFF\n");

	// Integration is not really appropriate in this controller
	// printw("do integration: %d\n", error_integration);

	printw("----------------------------------\n");
	printw("Gains (linear P I D, angular P I D, P-booster):\n\t");  
	PRINT_VECTOR(K)

	printw("Base state ([x,y,t,x.,y.,t.] in world frame):\n\t");
	PRINT_VECTOR(state)

	printw("Reference state ([x,y,t,x.,y.,t.] in world frame):\n\t");
	PRINT_VECTOR(refstate)

	printw("Wheel State (:\n\t");
	PRINT_VECTOR(wheel_state)

	printw("Trajectory index: [%d/%d]\n", trajIdx, trajectory.size());
}

void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&cx.d, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing data until stop received
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	int lastMode = mode;
	struct timespec t_forwStart;

	while(!somatic_sig_received) {

		pthread_mutex_lock(&mutex);
		// dbg = (c_++ % 20 == 0);
		// if(dbg) cout << "\nmode: " << mode;
		// if(dbg) cout << " start: " << start;
		// if(dbg) cout << " advance waypoints: " << advance_waypts;
		// if(dbg) cout << " do integration: " << error_integration << endl;

		// Krang::curses_display_row = CURSES_DEBUG_DISPLAY_START;
		// clear();
		move(0, 0);
		printw("----------------------------------\n");
		print_key_bindings();
		printw("----------------------------------\n");
		print_status(krang, cx);
		printw("----------------------------------\n");
		printw("DEBUG MESSAGES\n");

		// Read the gains if requested by user
		if(update_gains) {
			readGains();
			update_gains = false;
		}

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// poll krang_vision for updates to state before odometry updates
		poll_vision_channel(false, mode < 3); // don't reset reference if executing traj

		// Get the state and update odometry
		last_wheel_state = wheel_state;
		updateWheelsState(wheel_state, dt); 
		updateState(wheel_state, last_wheel_state, state);
		// if (dbg) cout << "wheel_state: " << wheel_state.transpose() << endl;
		// if (dbg) cout << "state: " << state.transpose() << endl;

		// save trajectory tracking as requested
		// if(save_hist) {
		// 	fprintf(state_hist_file, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t0\n", 
		// 		state(0), state(1), state(2), 
		// 		refstate(0), refstate(1), refstate(2));
		// 	fflush(state_hist_file);
		// }


		// Check if a new trajectory data is given
		updateTrajectory();
		
		// Update the reference state if necessary
		if(mode == 3 && !trajectory.empty()) {
			Vector6d err = computeError(state, refstate, true);
			double lin_error = abs(err[0]); 				//< linear distance in controllable direction
			double rot_error = abs(err[2]); 				//< angular distance to current waypoint

			bool reached = ((lin_error < waypt_lin_thresh)) && (rot_error < waypt_rot_thresh);
			
			// if(dbg) cout << "traj idx lin_error: " << (lin_error) << ", vs. " << waypt_lin_thresh  << endl;
			// if(dbg) cout << "traj idx rot_error (deg): " << R2D(rot_error) << ", vs. " << R2D(waypt_rot_thresh) << endl;
			// if(dbg) printf("reached: %d, xreached: %d, threached: %d\n", reached,
			// 	(lin_error < waypt_lin_thresh), (rot_error < waypt_rot_thresh));
			
			if(dbg) {
				printw("lin_error: %f (thresh = %f)\n", lin_error, waypt_lin_thresh); 
				printw("rot_error: %f (thresh = %f)\n", R2D(rot_error), R2D(rot_error)); 
				printw("reached: %d, xreached: %d, threached: %d\n", reached,
					(lin_error < waypt_lin_thresh), (rot_error < waypt_rot_thresh));
			} 

			if (reached) {
				// reset integral stuff whenever we reach any reference
				ang_int_err = 0;
				lin_int_err = 0;

				if (advance_waypts) {
					// advance the reference index in the trajectory, or stop if done		
					trajIdx = min((trajIdx + 1), (trajectory.size() - 1));
					setReference(trajectory[trajIdx]);

					if(trajIdx == trajectory.size() - 1) {
						mode = 1;
						error_integration = false;
					}
				}
			}
		}

		// if(dbg) cout << "traj idx: " << trajIdx << ", traj size: " << trajectory.size() << endl;
		// if(dbg) cout << "refstate: " << refstate.transpose() << endl;

		// Send the state
		if(true) {
			char buf[56]; // two ints and 6 doubles
			serialize_from_Matrix(state.transpose(), buf); // send state as row-vector like vision
			ach_put(&state_chan, &buf, 56);
		}

		// Compute the torques based on the state and the mode
		double ul, ur;
		computeTorques(state, ul, ur, dt);

		// Apply the torque
		double input[2] = {ul, ur};
		if(!start) input[0] = input[1] = 0.0;
		if (dbg) printw("sending wheel velocities: [%f, %f]", input[0], input[1]);

		somatic_motor_cmd(&cx.d, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
		lastMode = mode;

		refresh();  // refresh the ncurses screen
		pthread_mutex_unlock(&mutex);
	}

	// Send the stopping event
	somatic_d_event(&cx.d, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

void parse_args(int argc, char* argv[]) 
{
	memset(&cx, 0, sizeof(cx));

	// default options
	cx.opt_base_state_chan_name = "krang_base_state";
	cx.opt_base_pose_chan_name = "vision_krang_pose";
	cx.opt_base_waypts_chan_name = "krang_base_waypts";
	cx.d_opts.ident = "basenavd";
	cx.d_opts.sched_rt = SOMATIC_D_SCHED_UI;
	cx.opt_verbosity = 0;

	argp_parse(&argp, argc, argv, 0, NULL, &cx);

	// initialize as somatic daemon
	somatic_d_init(&cx.d, &cx.d_opts);

	if( cx.opt_verbosity ) {
		fprintf(stderr, "\n* basenavd *\n");
		fprintf(stderr, "Verbosity:    %d\n", cx.opt_verbosity);
		fprintf(stderr, "krang base state channel:      %s\n", cx.opt_base_state_chan_name);
		fprintf(stderr, "krang base pose channel:      %s\n", cx.opt_base_pose_chan_name);
		fprintf(stderr, "krang base waypoints channel:      %s\n", cx.opt_base_waypts_chan_name);
		fprintf(stderr,"-------\n");
   }
}

/* ******************************************************************************************** */
void init()
{
	ang_int_err = 0;
	lin_int_err = 0;
	error_integration = false;

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &cx.d, robot); 

	// Open channel to publish the state
	enum ach_status r = ach_open(&state_chan, cx.opt_base_state_chan_name, NULL);
	assert(ACH_OK == r);
	r = ach_flush(&state_chan);

	// Open channel to receive vision data
	r = ach_open(&vision_chan, cx.opt_base_pose_chan_name, NULL );
	assert(ACH_OK == r);
	r = ach_flush(&vision_chan);

	// Open channel to receive waypoints data
	r = ach_open(&base_waypts_chan, cx.opt_base_waypts_chan_name, NULL );
	assert(ACH_OK == r);
	r = ach_flush(&base_waypts_chan);

	// Receive the current state from vision
	if (wait_for_global_vision_msg_at_startup) {
		cout << "waiting for initial vision data on vision channel: " << endl;
		poll_vision_channel(true, false);
	}

	// Set the state, refstate and limits
	updateWheelsState(wheel_state, 0.0);
	last_wheel_state = wheel_state;
	updateState(wheel_state, last_wheel_state, state);
	setReference(state);

	// general curses stuff
	initscr();
	clear();
	noecho();                   // do not echo input to the screen
	cbreak();                   // do not buffer by line (receive characters immediately)
	timeout(0);                 // non-blocking getch

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

void destroy()
{
	// Destroy the daemon and the robot
	somatic_d_destroy(&cx.d);
	delete krang;

	Krang::destroy_curses();   // close display
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	// if (argc > 1) {
	// 	string flag = argv[1];
	// 	cout << "flag: " << flag << endl;
	// 	if (flag == "-v") {
	// 		wait_for_global_vision_msg_at_startup = true;
	// 	}
	// }
	
	// parse command line arguments
	parse_args(argc, argv);

	// Read the gains
	readGains();

	// Load the world and the robot
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton(0);

	// Initialize the daemon and the drivers
	init();
	
	// run the main loop
	run();

	// clean up and exit
	destroy();

	return 0;
}
