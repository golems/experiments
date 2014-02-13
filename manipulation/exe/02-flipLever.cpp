/**
 * @file 02-flipLever.cpp
 * @author Can Erdogan
 * @date Jan 24, 2014
 * @brief Force-torque compensated balancing while exerting torque with the waist joint.
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
FILE* file;

ach_channel_t waistd_chan;
size_t n_modules = 2;
Somatic__MotorCmd *cmd_msg = somatic_motor_cmd_alloc(n_modules);

static const double wheelRadius = 0.264;
double fx = 0.0, fz = 0.0;
double totalMass;							///< the total mass of the robot
bool debugGlobal = false;
bool respond = true, balance = false;
bool dbg = false;
bool zero = false;						//< zero the ft sensor
double kp = 450, kd = 80; 

/* ********************************************************************************************* */
/// Get the torque input
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='r') respond = !respond;
		else if(input=='b') balance = !balance;
		else if(input=='z') zero = true;
		else if(input=='k') kp -= 10.0;
		else if(input=='i') kp += 10.0;
		else if(input=='j') kd -= 5.0;
		else if(input=='l') kd += 5.0;
		
	}
}

/* ******************************************************************************************** */
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "02-flipLever";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL, &daemon_cx, robot); 
	cout << "Created Krang representation" << endl;
	usleep(1e6);

	// Compute the total mass based on the urdf file
	totalMass = 0.0;
	for(size_t i = 0; i < robot->getNumNodes(); i++) totalMass += robot->getNode(i)->getMass();

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

	// Open ACH channel for the waist commands to print
	int r  = ach_open( &waistd_chan, "waistd-cmd", NULL );
	aa_hard_assert(r == ACH_OK,
		"Ach failure %s on opening waistd channel (%s, line %d)\n",
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
}

/* ******************************************************************************************** */
/// Computes the torque around the wheel axis due to the external force at current frame
double computeExternalTorque () {

	// Get the position of the gripper and remove the height of the wheel
	Eigen::Vector3d contact = robot->getNode("lGripper")->getWorldTransform().topRightCorner<3,1>();
	contact(2) -= wheelRadius;
	// if(dbg) cout << "\tcontact: " << contact.transpose() << endl;

	// Project the contact-axis vector to the x and z axes
	double xproj = contact.dot(Eigen::Vector3d(0.0, 0.0, 1.0));
	double zproj = 0.0; // contact.dot(Eigen::Vector3d(1.0, 0.0, 0.0)) - 0.2;
	// if(dbg) printf("\txproj: %lf, zproj: %lf\n", xproj, zproj);

	// Compute the torque due to each axes
	double xtorque = fx * xproj;	
	double ztorque = fz * zproj;	
	// if(dbg) printf("\txtorque: %lf, ztorque: %lf\n", xtorque, ztorque);
	
	// Return the sum
	return xtorque + ztorque;
}

/* ******************************************************************************************** */
/// Updates the balancing angle reference based on the input torque
double updateRef (const Eigen::Vector3d& com, double externalTorque) {

	// Compute the x component of the desired com
	double com_x = externalTorque / (totalMass * 9.81);
	if(!respond) com_x = 0.0;

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
	com(2) -= wheelRadius;
	com(0) += 0.0076 - 0.0052 - 0.0017 - 0.0030 - 0.0068 + 0.0175 + 0.003 - 0.007;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2));
	state(1) = krang->imuSpeed;
}

/* ******************************************************************************************** */
double computeTorques(const Eigen::Vector2d& refState, const Eigen::Vector2d& state) {

	// Compute the input to control the balancing angle
	double u_th = kp * (state(0) - refState(0)) + kd * (state(1) - refState(1));
	if(dbg) cout << "th err: " << state(0) - refState(0) << endl;

	// Return total!
	if(dbg) cout << "u_th: " << u_th << endl;
	return u_th;
}

/* ******************************************************************************************** */
double getWaistCmd () {

	// Wait to receive a new mssage on the waistd-ach channel for 1/10 seconds
	struct timespec currTime;
	clock_gettime( CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/100.0), currTime);
	int r;
	Somatic__WaistCmd *waistd_msg = SOMATIC_WAIT_LAST_UNPACK( r, somatic__waist_cmd, 
		&protobuf_c_system_allocator, 4096, &waistd_chan, &abstime );
	aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME,
			"Ach failure %s on waistd data receive (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// If message was received, set the torso_direction and current_mode according to the command
	if(r == ACH_OK || r==ACH_MISSED_FRAME) {
		switch(waistd_msg->mode) {
			case SOMATIC__WAIST_MODE__REAL_CURRENT_MODE: {
				double real_current = waistd_msg->data->data[0];
				if(real_current < 0.0) real_current = fmax(real_current, -14.5);	
				else real_current = fmin(real_current, 14.5);		
				return real_current;
			} break;
			default: return 0.0;
		}	
		somatic__waist_cmd__free_unpacked( waistd_msg, &protobuf_c_system_allocator );
	}

	// If message was not received, stop the waist motors  
	else if (r == ACH_TIMEOUT) return 0.0;
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
	Krang::Vector6d leftFTData, temp;
	leftFTData << 0,0,0,0,0,0;
	size_t leftFTIter = 0;
	const size_t numResetFTIters = 30;
	double lastWaistCmd = 0.0;
	while(!somatic_sig_received) {

		dbg = (c_++ % 20 == 0);

		// Accumulate data for left f/t sensor
		if(zero) {
			if(dbg)	cout << "Resetting FT" << endl;

			// Get the data
			if(krang->fts[Krang::LEFT]->getRaw(temp) && leftFTIter < numResetFTIters)  {	
				leftFTData += temp;	
				leftFTIter++; 
			}
		
			// If done accumulating, data compute the average and record it as offset
			if(leftFTIter == numResetFTIters) {
				leftFTData /= numResetFTIters;
				krang->fts[Krang::LEFT]->error(leftFTData, krang->fts[Krang::LEFT]->offset, false);
				leftFTData << 0,0,0,0,0,0; 
				leftFTIter = 0;
				zero = false;
			}
		} 

		fx = fz = 0.0;
		if(fabs(krang->fts[Krang::LEFT]->lastExternal(0)) > 3.0)
			fx = krang->fts[Krang::LEFT]->lastExternal(0);
		if(fabs(krang->fts[Krang::LEFT]->lastExternal(2)) > 3.0)
			fz = krang->fts[Krang::LEFT]->lastExternal(2);

		if(dbg) cout << "\nrespond: " << respond << endl;
		if(dbg) cout << "kp: " << kp << ", kd: " << kd << endl;
		if(dbg) cout << "fx: " << fx << ", fz: " << fz << endl;
		if(dbg) cout << "last waist cmd: " << lastWaistCmd << endl;

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;

		// Get the state and print to file
		getState(state, dt, com);
		lastWaistCmd = getWaistCmd();
		if(c_ % 10 == 0) 
			fprintf(file, "imu: %lf, waist: %lf, lft; <%lf, %lf, %lf>, wheel torque: %lf, waist current: %lf\n", 
			krang->imu, krang->waist->pos[0],  
			krang->fts[Krang::LEFT]->lastExternal(0),
			krang->fts[Krang::LEFT]->lastExternal(1),
			krang->fts[Krang::LEFT]->lastExternal(2),
			krang->amc->cur[0], 
			lastWaistCmd);
		if(c_ % 100 == 0) fflush(file);
		if(dbg) cout << "com: " << com.transpose() << endl;

		// Compute the average torque
		externalTorques[c_ % kNumTorques] = computeExternalTorque();
		double averageTorque = 0.0;
		for(size_t i = 0; i < kNumTorques; i++) averageTorque += externalTorques [i];
		averageTorque /= kNumTorques;

		// Compute the balancing angle reference based on the commanded torque 
		refState(0) = updateRef(com, averageTorque);
		if(dbg) cout << "th ref: "<< refState(0) << endl;

		// Compute the torques based on the state and the desired torque
		double u = computeTorques(refState, state);
		if(u > 50.0) u = 50.0;
		if(u < -50.0) u = -50.0;

		// Apply the torque
		if(dbg) cout << "u: " << u << ", balance: " << balance << endl;
		if(!balance) u = 0.0; 
		double input [2] = {u, u};
		somatic_motor_cmd(&daemon_cx,krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
	}

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// Open the file
	file = fopen("results.txt", "w+");
	fprintf(file, "wtf\n");


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
	fclose(file);
	return 0;
}
