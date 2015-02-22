/**
 * @file 04-trajectory.cpp
 * @author Can Erdogan, Jon Scholz
 * @date April 09, 2014
 * @brief Implements PID control to follow a trajectory. The global state of the robot is 
 * kept by combining vision data and odometry. If vision data is received, odometry is reset 
 * to stop error built-ups. 
 * WAS:--------- The state is the 6x1 (x,x.,y,y.,th,th.) of the robot where (x,y,th) is the transform that -------------
 The state is the 6x1 (x,y,theta,xdot,ydot,thetadot) of the robot where (x,y,theta) is the transform that
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
bool update_gains = false; 	//< if true, read new gains from the text file
size_t mode = 0;			//< 0 sitting, 1 keyboard commands, 2 trajectory following
bool advance_waypts = true;
bool wait_for_global_vision_msg_at_startup = false;
bool use_steering_method = false;
const double waypt_lin_thresh = 0.01; // 0.003; // 0.001;
const double waypt_rot_thresh = 0.015; // 0.0036;
bool error_integration = false;
double ang_int_err;					//< accumulator for angular error
double lin_int_err;					//< accumulator for linear error (x in robot frame)

/* ******************************************************************************************** */
typedef Eigen::Matrix<double,6,1> Vector6d;
Vector6d refstate;					//< reference state for controller (x,y,th,x.,y.,th.)
Vector6d state;						//< current state (x,y,th,x.,y.,th.)
Eigen::Vector4d wheel_state;		//< wheel pos and vels in radians (lphi, lphi., rphi, rphi.)
Eigen::Vector4d last_wheel_state; 	//< last wheel state used to update the state 
vector <Vector6d> trajectory;		//< the goal trajectory	
size_t trajIdx = 0;
FILE* file;							//< used to print the state when the next trajectory index is used

/* ******************************************************************************************** */
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;	//< mutex to update gains
Eigen::VectorXd K = Eigen::VectorXd::Zero(7);	//< the gains for x and th of the state. y is ignored.

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
	double rtraj[1][3] = {0, 0, 0};

	size_t frame_size = 0;
	int options = block ? ACH_O_WAIT : ACH_O_LAST;
	int r = ach_get(&vision_chan, &rtraj, sizeof(rtraj), &frame_size, NULL, options); 
	if(!(r == ACH_OK || r == ACH_MISSED_FRAME)) 
		return;
	// for(size_t i = 0; i < 3; i++) 
	// 	printf("%lf\t", rtraj[0][i]);
	// printf("\n");

	// Set the current state to vision data
	for(size_t i = 0; i < 3; i++) 
		state(i) = rtraj[0][i];
	state(3) = state(4) = state(5) = 0.0; // set vels to zero (will be over-written by odometry)
	cout << "Updated state from vision message: " << state.transpose() << endl;

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

	//JFU: turn on error integration whenever we update the reference
	// error_integration = true;
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
	double vleft = krang->amc->vel[0] * wheel_diameter;  //< left wheel velocity in m/s
	double vright = krang->amc->vel[1] * wheel_diameter; //< right wheel velocity in m/s

	// Set the state
	wheel_state(0) = k2*(tleft + tright)/2.0;// + krang->imu;
	// wheel_state(1) = k2*(vleft + vright)/2.0;// + krang->imuSpeed; //<--- was noisy b/c amc vel is noisy
	wheel_state(1) = (wheel_state[0] - last_wheel_state[0]) * dt;
	wheel_state(2) = k1*(tright - tleft)/width; // (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	// wheel_state(3) = k1*(vright - vleft)/width; // TODO: verify 	//<--- was noisy b/c amc vel is noisy
	wheel_state(3) = (wheel_state[3] - last_wheel_state[3]) * dt;
}

/* ******************************************************************************************** */
/// Update the state using the wheels information with Euler approximation (we use the last
/// theta with the forward motion to reflect changes on x and y axes)
/*
 * state is a 6-vector containing:
 ---* 0: x
 ---* 1: xdot
 ---* 2: y
 ---* 3: ydot
 ---* 4: theta
 ---* 5: thetadot
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
	// double last_theta = state[4]; // last robot heading is whatever theta was before we update it
	double last_theta = state[2]; // last robot heading is whatever theta was before we update it

	// state[0] = state[0] + dlin * cos(last_theta);//< x
	// state[1] = wheel_state[1]; 					//< xdot <--- this is wrong; wheel_state is in robot frame but state is in world (odom) frame!
	// state[2] = state[2] + dlin * sin(last_theta);//< y
	// state[3] = 0;								//< ydot <--- this is wrong; there is no y velocity in the robot frame, but state is in world (odom) frame!
	// state[4] = state[4] + dtheta;				//< theta
	// state[5] = wheel_state[3];					//< thetadot

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

	// cout << "err: " << err.transpose() << endl;
	return err;
}

/* ******************************************************************************************** */
void computeTorques(const Vector6d& state, double& ul, double& ur, double& dt) {
	// Set reference based on the mode
	if(mode == 0) {
		ul = ur = 0.0;
		return;
	}

	// // Compute the linear pos error by projecting the reference state's position in the current
	// // state frame to the current heading direction
	// // Eigen::Vector2d dir (cos(state(4)), sin(state(4)));
	// // Eigen::Vector2d refInCurr (refstate(0) - state(0), refstate(2) - state(2));
	// Eigen::Vector2d dir (cos(state(2)), sin(state(2)));
	// Eigen::Vector2d linear_pos_err_2D (refstate(0) - state(0), refstate(1) - state(1)); // dx,dy for ref-cur

	// double linear_pos_err = dir.dot(linear_pos_err_2D); // dot product of current heading and linear error
	// // double linear_vel_err = state(1); // we want zero linear velocity
	// // double linear_vel_err = state(3); // we want zero linear velocity;  but in world frame this is wrong now!  should be state[3:4].dot(state[3:4])
	// Eigen::Vector2d planar_vel (state(3), state(4));
	// double linear_vel_err = planar_vel.dot(planar_vel);

	// // Compute the angular position error by taking the difference of the two headings
	// // double angular_pos_err = refstate(4) - state(4); // Gah gotta unwrap this
	// double angular_pos_err = unwrap(refstate(2) - state(2)); 
	// double angular_vel_err = state(5); // want zero rotational velocity

	// // Compute the error and set the y-components to 0
	// Eigen::Vector4d error (linear_pos_err, linear_vel_err, angular_pos_err, angular_vel_err);
	// if(dbg) cout << "error: " << error.transpose() << endl;
	// if(dbg) cout << "[K: " << K.transpose() << "]" << endl;

	// if (error_integration) {
	// 	ang_int_err += angular_pos_err * dt;
	// 	lin_int_err += linear_pos_err * dt;
	// }

	// if (dbg) cout << "ang_int_err: " << ang_int_err << " lin_int_err: " << lin_int_err << endl;

	// New version:
	
	// // Error vector in world (or odom) frame, which is where state and refstate are defined
	// Vector6d err_world = refstate - state;
	// err_world[2] = unwrap(err_world[2]);
	// // cout << "err_world: " << err_world.transpose() << endl;


	// // strict steering method:
	// if (false) {
	// 	double refpos_angle = atan2(err_world[1], err_world[0]); 	//< angle towards current waypoint
	// 	double refpos_dist = err_world.segment(0,2).norm(); 		//< linear distance to current waypoint
	// 	double refpos_angle_err = unwrap(state[2] - refpos_angle); 	//< heading error 
	// 	if (refpos_dist > 5e-2)
	// 		err_world[2] = refpos_angle_err; 						//< if we're far from the waypoint only worry about heading towards it
	// 	if (refpos_angle_err > 1e-2)
	// 		err_world.segment(0,2).setZero(); 						//< zero all translation controls if we're rotating
	// }
	Vector6d err_robot = computeError(state, refstate, true);

	// HACK: add static-friction compensation term to get robot started
	// double p_booster = 1.0;
	// double vel_thresh = 0.01;
	// if (state.segment(3,2).squaredNorm() < vel_thresh)
	// 	p_booster = K[6]; 

	double velsq = state.segment(3,2).squaredNorm();
	double p_booster = K[6] * exp(-20 * velsq); //< exponentially falls off as robot gains speed

	if (error_integration) {
		lin_int_err += err_robot[0] * dt;		//< only integrate error in direction we can control
		ang_int_err += err_robot[2] * dt;	//< integrate angular error
	}

	if (dbg) cout << "error: " << err_robot.transpose() << endl;
	if (dbg) cout << "ang_int_err: " << ang_int_err << " lin_int_err: " << lin_int_err << endl;

	// // Compute the forward and spin torques (note K is 4x1 for x and th)
	// // note: K is organized as [linearP linearI linearD angularP angularI angularD]
	// // 		 err_robot is organized as [x xdot theta thetadot]
	// double u_x = (K[0]*err_robot(0) + K[2]*err_robot(1)) + K[1] * lin_int_err;
	// double u_theta = (K[3]*err_robot(2) + K[5]*err_robot(3)) + K[4] * ang_int_err;

	// Compute the forward and rotation torques
	// note: K is organized as [linearP linearI linearD angularP angularI angularD]
	// 		 err_robot is organized as [x, y, theta, xdot, ydot, thetadot]
	double u_x 		= (K[0] * err_robot[0] * p_booster + K[1] * lin_int_err + K[2] * err_robot[3]);
	double u_theta 	= (K[3] * err_robot[2] * p_booster + K[4] * ang_int_err + K[5] * err_robot[5]);

	// Limit the output torques
	double hard_max = 15.0;		//< never write values higher than this
	double spin_max = 15.0; 	//< thershold on spin contribution
	double lin_max = 15.0; 		//< threshold on linear contribution
	u_theta = max(-spin_max, min(spin_max, u_theta));
	u_x= max(-lin_max, min(lin_max, u_x));
	if(dbg) printf("u_x: %lf, u_theta: %lf\n", u_x, u_theta);
	ul = u_x - u_theta;
	ur = u_x + u_theta;
	ul = max(-hard_max, min(hard_max, ul));
	ur = max(-hard_max, min(hard_max, ur));
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
		Vector6d newRefState;
		//newRefState << rtraj[p_idx][0], 0.0, rtraj[p_idx][1], 0.0, rtraj[p_idx][2], 0.0;
		newRefState << rtraj[p_idx][0], rtraj[p_idx][1], rtraj[p_idx][2], 0.0, 0.0, 0.0;
		trajectory.push_back(newRefState);
	}

	// Update the reference state
	trajIdx = 0;
	setReference(trajectory[0]);

	// turn on trajectory following mode
	mode = 2;
}


/// sets the trajectory to some hard-coded thing for debugging
void setTestTrajectory()
{
	// clear the trajectory
	trajectory.clear();
	
	// push back some waypoints
	double x0 = state[0];
	// trajectory.push_back((Eigen::VectorXd(6) << x0, 0, 0, 0, 0, 0).finished());
	// trajectory.push_back((Eigen::VectorXd(6) << x0 +0.1, 0, 0, 0, 0, 0).finished());
	// trajectory.push_back((Eigen::VectorXd(6) << x0 +0.2, 0, 0, 0, 0, 0).finished());
	// trajectory.push_back((Eigen::VectorXd(6) << x0 +0.3, 0, 0, 0, 0, 0).finished());

	trajectory.push_back((Eigen::VectorXd(6) << x0, 0, 0, 0, 0, 0).finished());
	trajectory.push_back((Eigen::VectorXd(6) << x0 + 0.2, 0, 0.1, 0, 0, 0).finished());
	trajectory.push_back((Eigen::VectorXd(6) << x0 + 0.4, 0, 0.2, 0, 0, 0).finished());
	trajectory.push_back((Eigen::VectorXd(6) << x0 + 0.2, 0, 0.1, 0, 0, 0).finished());
	trajectory.push_back((Eigen::VectorXd(6) << x0, 0, 0, 0, 0, 0).finished());

	// Update the reference state
	trajIdx = 0;
	setReference(trajectory[0]);
	mode = 2;
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
		if(dbg) cout << " advance waypoints: " << advance_waypts;
		if(dbg) cout << " do integration: " << error_integration << endl;
	
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
		poll_vision_channel(false, mode < 2); // don't reset reference if executing traj

		// Get the state and update odometry
		last_wheel_state = wheel_state;
		updateWheelsState(wheel_state, dt); 
		if (dbg) cout << "wheel_state: " << wheel_state.transpose() << endl;
		updateState(wheel_state, last_wheel_state, state);
		if(dbg) {
			fprintf(file, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t0\n", 
				state(0), state(1), state(2), 
				refstate(0), refstate(1), refstate(2));
			fflush(file);
		}
		if(dbg) cout << "state: " << state.transpose() << endl;

		// Check if a new trajectory data is given
		updateTrajectory();
		
		// Update the reference state if necessary
		if(mode == 2 && !trajectory.empty()) {
			// // Eigen::Vector2d dir (cos(state(4)), sin(state(4)));
			// Eigen::Vector2d dir (cos(state(2)), sin(state(2)));
			// // Eigen::Vector2d refInCurr (refstate(0) - state(0), refstate(2) - state(2));
			// Eigen::Vector2d refInCurr (refstate(0) - state(0), refstate(1) - state(1));
			// double xerror = dir.dot(refInCurr);
			// // double thetaerror = SQ(refstate(4) - state(4));
			// double thetaerror = SQ(unwrap(refstate(2) - state(2)));

			Vector6d err = computeError(state, refstate, true);
			// double lin_error = err.segment(0,2).norm(); 	//< linear distance to current waypoint
			double lin_error = abs(err[0]); 				//< linear distance in controllable direction
			double rot_error = abs(err[2]); 				//< angular distance to current waypoint

			if(dbg) cout << "traj idx lin_error: " << (lin_error) << ", vs. " << waypt_lin_thresh  << endl;
			if(dbg) cout << "traj idx rot_error (deg): " << R2D(rot_error) << ", vs. " << R2D(waypt_rot_thresh) << endl;
			// bool reached = (trajIdx < 2 || (lin_error < waypt_lin_thresh)) && (rot_error < waypt_rot_thresh);
			bool reached = ((lin_error < waypt_lin_thresh)) && (rot_error < waypt_rot_thresh);

			if(dbg) printf("reached: %d, xreached: %d, threached: %d\n", reached,
				(lin_error < waypt_lin_thresh), (rot_error < waypt_rot_thresh));

			if (reached) {
				// reset integral stuff whenever we reach any reference
				ang_int_err = 0;
		        lin_int_err = 0;

		        //JFU:
		        /*
		         * Kill integration whenever we reach a reference.  It gets turned back
		         * on whenever we set a new reference.
		         * If we don't do this, residual error from the waypoint eventualy builds
		         * up and the robot starts to move again!
		         */
		        // error_integration = false;

		        if (advance_waypts) {
					/*
					 * advance the reference index in the trajectory, or stop if done
					 */
					// fprintf(file, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t1\n", state(0), state(2), state(4),
					// 	refstate(0), refstate(2), refstate(4));
					fprintf(file, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t1\n", state(0), state(1), state(2),
						refstate(0), refstate(1), refstate(2));
					fflush(file);
					trajIdx = min((trajIdx + 1), (trajectory.size() - 1));
					setReference(trajectory[trajIdx]);

					if(trajIdx == trajectory.size() - 1) {
						mode = 0;
						error_integration = false;
					}
		        }
			}

		}
		if(dbg) cout << "traj idx: " << trajIdx << ", traj size: " << trajectory.size() << endl;
		if(dbg) cout << "refstate: " << refstate.transpose() << endl;

		// Send the state
		if(true) {
		  double traj[1][6] = {{state(0), state(1), state(2), state(3), state(4), state(5)}};
		  //if(dbg) cout << "sending state: " << state.transpose() << endl;
		  ach_put(&state_chan, &traj, sizeof(traj));
		}

		// Compute the torques based on the state and the mode
		double ul, ur;
		computeTorques(state, ul, ur, dt);

		// Apply the torque
		double input[2] = {ul, ur};
		if(!start) input[0] = input[1] = 0.0;
		if(dbg) cout << "sending wheel control input: {" << input[0] << ", " << input[1] << "}" << endl;
		somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
		lastMode = mode;
		pthread_mutex_unlock(&mutex);
	}

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
/// Get the mode input
/*
Keybindings:
'1' - mode: 
*/
void *kbhit(void *) {
	char input;
	
	while(true){ 
		input=cin.get(); 
		pthread_mutex_lock(&mutex);
		if(input=='0') mode = 0;
		else if(input=='1') mode = 1;
		else if(input=='2') mode = 2;
		else if(input=='d') dbg = !dbg;
		else if(input=='s') start = !start;
		else if(input=='g') update_gains = true; 	//< if true, read new gains from the text file
		else if(input=='r') { 								//< set reference to current state at next iteration
			setReference(state);
			trajectory.clear();
			ang_int_err = 0;
			lin_int_err = 0;
		}
		else if(input=='i') { 
			// input is in the the robot frame, so we rotate to world for refstate
			refstate(0) += 10*waypt_lin_thresh * cos(state[2]);
			refstate(1) += 10*waypt_lin_thresh * sin(state[2]);
		}
		else if(input=='k') {
			// input is in the the robot frame, so we rotate to world for refstate
			refstate(0) -= 10*waypt_lin_thresh * cos(state[2]);
			refstate(1) -= 10*waypt_lin_thresh * sin(state[2]);
		}
		else if(input=='j') refstate(2) += 10*waypt_rot_thresh;
		else if(input=='l') refstate(2) -= 10*waypt_rot_thresh;
		else if(input=='o') error_integration = !error_integration;
		else if(input==' ') advance_waypts = !advance_waypts;
		else if(input=='t') setTestTrajectory();
		pthread_mutex_unlock(&mutex);
	}
}

/* ******************************************************************************************** */
void init() {
	ang_int_err = 0;
    lin_int_err = 0;
    error_integration = false;

	// Open the file
	file = fopen("bla", "w+");
	assert((file != NULL) && "Could not open the file");

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "05-trajectoryPID";
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
	if (wait_for_global_vision_msg_at_startup) {
		cout << "waiting for initial vision data on [krang_vision]: " << endl;
		poll_vision_channel(true, false);
	}

	// Set the state, refstate and limits
	updateWheelsState(wheel_state, 0.0);
	last_wheel_state = wheel_state;
	updateState(wheel_state, last_wheel_state, state);
	
	setReference(state);

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

    if (argc > 1) {
    	string flag = argv[1];
    	cout << "flag: " << flag << endl;
    	if (flag == "-v") {
    		wait_for_global_vision_msg_at_startup = true;
    	}
    }
    
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
