/**
 * @file 06-fx.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date Jan 09, 2014
 * @brief Assuming that the front of the robot faces the +x direction, we'd like to control the
 * force it exerts in that direction. However, we do not want to use the mass of the robot!
 * 
 * Assume fx_external is towards the robot.
 *
 * The idea is the following two equations:
 * (1) fx_net = (2*tau/wheelRadius) - fx_external = 0
 * (2) torq_y_net = mgrsin(th) + l*fx_external*cos(th + k) + l*fx_external*cos(th + k) - 2*tau = 0
 * where r and l are the distances of COM and contact point to wheel axis, k is the angle
 * between the com line and the contact line through the axis, and m is the robot mass.
 * 
 * For this application, we assume external contact. The goal is to provide goal fx and fz values
 * and see if the robot can achieve them. We first compute tau from the first equation and 
 * then compute th from the second. Hopefully, the system can converge to the desired values.
 * Note that we assume that the robot can change its pose while in contact - we have seen that
 * due to friction forces with the contact, it might not reach its goal theta. If that's so,
 * we might play with the kp and kd - and even add some ki.
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

static const double wheelRadius = 0.264;
double fx = 0.0, fz = 0.0;
double totalMass;							///< the total mass of the robot
bool debugGlobal = false;
bool respond = true;
bool dbg = false;

/* ********************************************************************************************* */
/// Get the torque input
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='k') fx += 0.5;
		else if(input=='j') fx -= 0.5;
		else if(input=='h') fz += 0.5;
		else if(input=='l') fz -= 0.5;
		else if(input=='r') respond = !respond;
	}
}

/* ******************************************************************************************** */
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "06-fx";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot); 
	cout << "Created Krang representation" << endl;
	usleep(1e6);

	// Compute the total mass based on the urdf file
	totalMass = 0.0;
	for(size_t i = 0; i < robot->getNumNodes(); i++) totalMass += robot->getNode(i)->getMass();

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// Computes the torque around the wheel axis due to the external force at current frame
double computeExternalTorque () {

	// Get the position of the gripper and remove the height of the wheel
	Eigen::Vector3d contact = robot->getNode("lGripper")->getWorldTransform().topRightCorner<3,1>();
	contact(2) -= wheelRadius;

	// Project the contact-axis vector to the x and z axes
	double xproj = contact.dot(Eigen::Vector3d(1.0, 0.0, 0.0));
	double zproj = contact.dot(Eigen::Vector3d(0.0, 0.0, 1.0));

	// Compute the torque due to each axes
	double xtorque = fx * xproj;	
	double ztorque = fz * zproj;	
	
	// Return the sum
	return xtorque + ztorque;
}

/* ******************************************************************************************** */
/// Updates the balancing angle reference based on the input torque
pair <double, double> updateRef (const Eigen::Vector3d& com, double externalTorque) {

	// Compute the torque necessary to achieve fx - the reaction torque from the hweels
	double tau = fx * wheelRadius;
	if(!respond) tau = 0;

	// Compute the x component of the desired com
	double com_x = (externalTorque - tau) / (totalMass * 9.81);

	// Compute the z component of the desired com by first computing the norm (which is fixed)
	// and then computing the z from x component
	double normSq = com(0) * com(0) + com(2) * com(2);
	double com_z = sqrt(normSq - com_x * com_x);

	// Compute the expected balancing angle	
	return make_pair(atan2(-com_x, com_z), -tau);
}

/* ******************************************************************************************** */
/// Get the current state
void getState (Eigen::Vector2d& state, double dt, Eigen::Vector3d& com) {

	// Read motor encoders, imu and ft and update dart skeleton
  krang->updateSensors(dt);

	// Calculate the COM and make adjustments
	com = robot->getWorldCOM();
	com(2) -= wheelRadius;
	com(0) += 0.0070;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2));
	state(1) = krang->imuSpeed;
}

/* ******************************************************************************************** */
double computeTorques(const Eigen::Vector2d& refState, const Eigen::Vector2d& state, 
		double torque) {

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

	// Prepare the vector to average the forces to respond
	size_t kNumTorques = 100;
	vector <double> externalTorques;
	for(size_t i = 0; i < kNumTorques; i++) externalTorques.push_back(0.0);

	// Continue processing data until stop received
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	Eigen::Vector3d com;
	Eigen::Vector2d state, refState;
	refState(1) = 0.0;
	while(!somatic_sig_received) {

		dbg = (c_++ % 50 == 0);

		fx = fz = 0.0;
		if(fabs(krang->fts[Krang::LEFT]->lastExternal(0)) > 3.0)
			fx = krang->fts[Krang::LEFT]->lastExternal(0);
		if(fabs(krang->fts[Krang::LEFT]->lastExternal(2)) > 3.0)
			fz = krang->fts[Krang::LEFT]->lastExternal(2);

		if(dbg) cout << "\nrespond: " << respond << endl;
		if(dbg) cout << "requested fx: " << fx << ", fz: " << fz << endl;
		if(dbg) cout << "real      fx: " << krang->fts[Krang::LEFT]->lastExternal(0) << ", fz: " << 
			krang->fts[Krang::LEFT]->lastExternal(2) << endl;

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Get the state 
		getState(state, dt, com);
		if(dbg) cout << "com: " << com.transpose() << endl;

		// Compute the average torque
		externalTorques[c_ % kNumTorques] = computeExternalTorque();
		double averageTorque = 0.0;
		for(size_t i = 0; i < kNumTorques; i++) averageTorque += externalTorques [i];
		averageTorque /= kNumTorques;

		// Compute the balancing angle reference based on the commanded torque 
		pair <double, double> refs = updateRef(com, averageTorque);
		refState(0) = refs.first;
		if(dbg) cout << "th ref: "<< refState(0) << endl;

		// Compute the torques based on the state and the desired torque
		double u = computeTorques(refState, state, refs.second);
		if(u > 20.0) u = 20.0;
		if(u < -20.0) u = -20.0;

		// Apply the torque
		double input [2] = {u, u};
		if(dbg) cout << "u: " << u << endl;
		somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
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
