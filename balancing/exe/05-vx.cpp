/**
 * @file 05-vx.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date Jan 09, 2014
 * @brief Assuming that the front of the robot faces the +x direction, we'd like to control the
 * force it exerts in that direction. However, we do not want to use the mass of the robot!
 * 
 * The idea is the following two equations:
 * (1) fx_net = (2*tau/wheelRadius) + fx_external
 * (2) torq_y_net = mgrsin(th) + l*fx_external*cos(th + k) + l*fx_external*cos(th + k) + 2*tau
 * where r and l are the distances of COM and contact point to wheel axis, k is the angle
 * between the com line and the contact line through the axis, and m is the robot mass.
 * 
 * For this application, we assume no external contact and just want to see the robot accelerate
 * and decelerate in the +x direction using wheel torques, tau, with the torque around the y-axis
 * preserved around 0. That is the robot needs to shift its weight to counter the reaction torque
 * and not use it to move.
 */ 

#include <iostream>

#include <Eigen/Dense>

#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <initModules.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>

#include <kore.hpp>

using namespace std;

somatic_d_t daemon_cx;				///< The context of the current daemon
Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
simulation::World* world;			///< the world representation in dart
dynamics::SkeletonDynamics* robot;			///< the robot representation in dart

double torque = 0.0;					///< the torque applied by the wheels
double totalMass;							///< the total mass of the robot
bool debugGlobal = false;
bool dbg = false, balance = false;

/* ********************************************************************************************* */
/// Get the torque input
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='k') torque += 0.5;
		else if(input=='j') torque -= 0.5;
		else if(input=='b') balance = !balance;
	}
}

/* ******************************************************************************************** */
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "05-vx";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot); 

	// Compute the total mass based on the urdf file
	totalMass = 0.0;
	for(size_t i = 0; i < robot->getNumNodes(); i++) totalMass += robot->getNode(i)->getMass();

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// Updates the balancing angle reference based on the input torque
double updateRef (const Eigen::Vector3d& com) {

	// Compute the x component of the desired com
	double com_x = -torque / (totalMass * 9.81);

	// Compute the z component of the desired com by first computing the norm (which is fixed)
	// and then computing the z from x component
	double normSq = com(0) * com(0) + com(2) * com(2);
	double com_z = sqrt(normSq - com_x * com_x);

	// Compute the expected balancing angle	
	return atan2(-com_x, com_z);
}

/* ******************************************************************************************** */
/// Get the current state
void getState (Eigen::Vector2d& state, double dt, Eigen::Vector3d& com) {

	// Read motor encoders, imu and ft and update dart skeleton
  krang->updateSensors(dt);

	// Calculate the COM and make adjustments
	com = robot->getWorldCOM();
	com(2) -= 0.264;
	com(0) += 0.0076 + 0.0045;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2));
	state(1) = krang->imuSpeed;
}

/* ******************************************************************************************** */
double computeTorques(const Eigen::Vector2d& refState, const Eigen::Vector2d& state) {

	// Compute the input to control the balancing angle
	static const double kp = 250, kd = 40; 
	double u_th = kp * (state(0) - refState(0)) + kd * (state(1) - refState(1));
	if(dbg) cout << "th err: " << state(0) - refState(0) << endl;

	// Compute the input to cause the desired torque
	double u_torque = torque / 1.7;

	// Return total!
	if(dbg) cout << "u_th: " << u_th << ", u_torque: " << u_torque << endl;
	return u_th + u_torque;
}

/* ******************************************************************************************** */
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Continue processing data until stop received
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	Eigen::Vector3d com;
	Eigen::Vector2d state, refState;
	refState(1) = 0.0;
	while(!somatic_sig_received) {

		dbg = (c_++ % 20 == 0);
		if(dbg) cout << "\ntorque: " << torque << endl;

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Get the state
		getState(state, dt, com);
		if(dbg) cout << "com: " << com.transpose() << endl;

		// Compute the balancing angle reference based on the commanded torque 
		refState(0) = updateRef(com);
		if(dbg) cout << "th ref: "<< refState(0) << endl;

		// Compute the torques based on the state and the desired torque
		double u = computeTorques(refState, state);
		if(u > 20.0) u = 20.0;
		if(u < -20.0) u = -20.0;

		// Apply the torque
		double input [2] = {u, u};
		if(dbg) cout << "u: " << u << endl;
		if(balance) somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
	}

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

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
