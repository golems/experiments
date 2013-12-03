// Author: Xinyan Yan
// Date: 12/1/2013
// Use keyboard to do workspace control and also test whether fingers
// are in contact or not

#include <kore.hpp>
#include <kore/workspace.hpp>
#include <kore/display.hpp>
#include <kore/util.hpp>

#include <iomanip>
#include <iostream>
#include <fstream>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <somatic/msg.h>

#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>


using namespace std;


// Constants
// initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.50; // maximum 50 cm per second from spacenav
const double SPACENAV_TRANSLATION_GAIN = 0.25; // maximum .25 radians per second from spacenav
const double COMPLIANCE_TRANSLATION_GAIN = 1.0 / 750.0;
const double COMPLIANCE_ORIENTATION_GAIN = .125 / 750.0;
const double HAND_OVER_HAND_SPEED = 0.05; // 3 cm per second when going hand-over-hand

// various rate limits - be nice to other programs
const double LOOP_FREQUENCY = 100.0;
const double FINGER_LOOP_FREQUENCY = 200.0;
//const double LOOP_FREQUENCY = 100.0;
const double DISPLAY_FREQUENCY = 10.0;

// display information
const int CURSES_DEBUG_DISPLAY_START = 20;

double GRIPPER_POSITION_OPEN[] = {0, 0, 0, 128};
double GRIPPER_POSITION_CLOSE[] = {255, 255, 255, 128};
double GRIPPER_POSITION_PARTIAL[] = {70, 70, 70, 128};



// Hardware object
Krang::Hardware *hw;

// Process state
somatic_d_t daemon_cx;

// Pointers to important DART objects
simulation::World *world;
dynamics::SkeletonDynamics *robot;

// Workspace stuff
Krang::WorkspaceControl *wss;
Krang::Vector7d nullspace_q_refs;
Krang::Vector7d nullspace_q_masks;
Krang::Vector7d nullspace_qdot_refs;

// debug information
bool debug_print_this_it;       ///< whether we print


double FORCEMAGNTRESH = 6;
double MAXFORCE = 30;
const double SPEEDMAGN = 0.02;
const double TIME_LIMIT = 3;
const Eigen::Vector3d POSCHANGE(-0.1, 0, 0);
const Krang::Side TOUCHSIDE = Krang::LEFT;
const Krang::Side sde = TOUCHSIDE;
const double DIST_TOLERANCE = 0.001;
const double RAD_TOLERANCE = 0.05;
//const double MOVEDISTUNIT = 0.05;	// 5cm?
const double MOVEDISTUNIT = 0.02;	
//const double MOVERADUNIT = 0.09;	// about 5 degress?
const double MOVERADUNIT = 0.2;	


// For fingers
const double MAX_CLOSE_TIME = 2;
const double NEUTRAL_POSI[] = {70,70,70,128};
const double CLOSE_POSI[] = {200,200,200,128};
const double OPEN_POSI[] = {10,10,10,128};

double current_threshold = 13.5;
bool contact_close[] = {false, false, false, false};
bool contact_open[] = {false, false, false, false};
double position_close[4];
double position_open[4];


double resetAngles[] =  { 0.998,  -1.691,  -0.111,  -0.817,  -1.293,  -1.609,  -0.176};

double BALLOFFSET = 0.1235; // (4.05 + 2.7 + 5.6)cm

void MoveWorldBy(const Eigen::VectorXd &change, double maxForce, bool toRecord);
void MoveEEBy(const Eigen::VectorXd &change, double maxForce, bool toRecord);
void MoveTo(const Eigen::VectorXd &end, double maxForce, bool toRecord);
void setvzero(const Krang::Side sde);
void MoveFingerTo(const double[], bool);
void PrintInfo();
void Log(ostream &file);
void LogFTP(ostream &file);
void LogCPDFT(ostream &file, bool contact, int direction);
int GetDirection(const Eigen::Vector3d &d);
ofstream CONF;
ofstream FTPF;
ofstream CPDFTF;

// Print out current Finger state
static void printCurPosCont(double current[], double position[], bool contact[]);


// For input command from keyboard
static int tty_unbuffered(int);     
static void tty_atexit(void);
static int tty_reset(int);
static void tweak_init();

static struct termios save_termios;
static int  ttysavefd = -1;



/* ******************************************************************************************** */
/// Initialization
void init() {

	// Open log file
	CONF.open("log_file", ios::out|ios::trunc);
	FTPF.open("ftp_log", ios::out|ios::trunc);
	CPDFTF.open("cpdft_log", ios::out|ios::trunc);

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "MoveGripperTest";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL);
	hw = new Krang::Hardware(mode, &daemon_cx, robot);

	// Set up the workspace stuff
	wss = new Krang::WorkspaceControl(robot, sde, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
	                                                SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN,
	                                                COMPLIANCE_TRANSLATION_GAIN, COMPLIANCE_ORIENTATION_GAIN);

	// set up nullspace stuff
	// Right
//	nullspace_q_refs = (Krang::Vector7d()  << 0,  1.0, 0,  0.5, 0,  0.8, 0).finished();
//	nullspace_q_masks = (Krang::Vector7d() << 0,    0, 0,    1, 0,    0, 0).finished();

	// Left
	nullspace_q_refs = (Krang::Vector7d()   << 0, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_masks = (Krang::Vector7d()  << 0,    0, 0,    1, 0,    0, 0).finished();


	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);


	// 
	tweak_init();
}


/* ******************************************************************************************** */
/// Clean up
void destroy() {

	CONF.close();
	FTPF.close();
	CPDFTF.close();

	// close display
//	Krang::destroy_curses();

	// Stop the daemon
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
	

	// Clean up the workspace stuff
	delete wss;

	// Close down the hardware
	delete hw;

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// For storing command from keyboard
	char c;

	// Desired finger position to achieve
	double fingerGoal[4];
	for (int i = 0; i < 4; i++)
		fingerGoal[i] = NEUTRAL_POSI[i];

	bool fingerMove = false;
	bool closeFinger;

	// Init finger position
	somatic_motor_setpos(&daemon_cx, hw->grippers[sde], fingerGoal, 4);


	Eigen::VectorXd tomove = Eigen::VectorXd::Zero(6);

	while(!somatic_sig_received) {

		// New command from keyboard?
		// Reset goal
		if ( read(STDIN_FILENO, &c, 1) == 1) {
			tomove = Eigen::VectorXd::Zero(6);
		    switch (c) {

		        case 'a':
					tomove(0) += MOVEDISTUNIT;
					cout << "####################" << endl;
					cout << "Increase 0" << endl;
		            break;
		        case 'z':
					tomove(0) -= MOVEDISTUNIT;
					cout << "####################" << endl;
					cout << "Decrease 0" << endl;
		            break;
		
		        case 's':
					tomove(1) += MOVEDISTUNIT;
					cout << "####################" << endl;
					cout << "Increase 1" << endl;
		            break;
		        case 'x':
					tomove(1) -= MOVEDISTUNIT;
					cout << "####################" << endl;
					cout << "Decrease 1" << endl;
		            break;
		
		        case 'd':
					tomove(2) += MOVEDISTUNIT;
					cout << "####################" << endl;
					cout << "Increase 2" << endl;
		            break;
		        case 'c':
					tomove(2) -= MOVEDISTUNIT;
					cout << "####################" << endl;
					cout << "Decrease 2" << endl;
		            break;

		        case 'f':
					tomove(3) += MOVERADUNIT;
					cout << "####################" << endl;
					cout << "Increase 3" << endl;
		            break;
		        case 'v':
					tomove(3) -= MOVERADUNIT;
					cout << "####################" << endl;
					cout << "Decrease 3" << endl;
		            break;


		        case 'g':
					tomove(4) += MOVERADUNIT;
					cout << "####################" << endl;
					cout << "Increase 4" << endl;
		            break;
		        case 'b':
					tomove(4) -= MOVERADUNIT;
					cout << "####################" << endl;
					cout << "Decrease 4" << endl;
		            break;

		        case 'h':
					tomove(5) += MOVERADUNIT;
					cout << "####################" << endl;
					cout << "Increase 5" << endl;
		            break;
		        case 'n':
					tomove(5) -= MOVERADUNIT;
					cout << "####################" << endl;
					cout << "Decrease 5" << endl;
		            break;
	

		        case 'i':
					cout << "####################" << endl;
					cout << "Approaching NEUTRAL" << endl;
					fingerMove = true;
					for (int i = 0; i < 4; i++)
						fingerGoal[i] = NEUTRAL_POSI[i];
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], fingerGoal, 4);
		            break;
		        case 'u':
					cout << "####################" << endl;
					cout << "Approaching CLOSE" << endl;
					fingerMove = true;
					closeFinger = 1;
					for (int i = 0; i < 4; i++)
						fingerGoal[i] = CLOSE_POSI[i];
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], fingerGoal, 4);
		            break;

		        case 'o':
					cout << "####################" << endl;
					cout << "Approaching OPEN" << endl;
					fingerMove = true;
					closeFinger = 0;
					for (int i = 0; i < 4; i++)
						fingerGoal[i] = OPEN_POSI[i];
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], fingerGoal, 4);
		            break;
				case 'p':
					current_threshold += 1;
					cout << "Current threshold: " << current_threshold << endl;
					break;

				case ';':
					current_threshold -= 1;
					cout << "Current threshold: " << current_threshold << endl;
					break;
				// Log

				case 'l':
					cout << "Log to local file" << endl;
					Log(CONF);
					continue;

				case 'r':
					cout << "Record Force, Torque, and EE position to local file" << endl;
					LogFTP(FTPF); 
					continue;

				case 'y':
					cout << "z back" << endl;
					tomove(2) =  MOVEDISTUNIT;
//					MoveEEBy(tomove, MAXFORCE);
					MoveWorldBy(tomove, MAXFORCE, false);
					sleep(2);
					continue;


				case 'm':
					cout << "Carry out a sample along z axis" << endl;

					// Get current joint angles
					double jointAngles[7];
					cout << "Joint angles: ";
					for (int i = 0; i < 7; i++) {
						jointAngles[i] = hw->arms[sde]->pos[i];
						cout << jointAngles[i] << "   ";
					}
					cout << endl;

					// Move in z axis, stop when force exceeds some value
					tomove(2) = -1 * MOVEDISTUNIT;
//					MoveEEBy(tomove, FORCEMAGNTRESH);
					MoveWorldBy(tomove, FORCEMAGNTRESH, true);
					sleep(2);

					// Log force, torque, and ee position
//					cout << "Log force, torque and ee position" << endl;
//					LogFTP(FTPF);

					// Move back
					cout << "Joint angles: ";
					for (int i = 0; i < 7; i++) {
						cout << hw->arms[sde]->pos[i] << "   ";
					}
					cout << endl;

					cout << "Move back" << endl;
					tomove(2) =  MOVEDISTUNIT;
//					MoveEEBy(tomove, MAXFORCE);
					MoveWorldBy(tomove, MAXFORCE, false);
					sleep(2);

//					somatic_motor_cmd(&daemon_cx, hw->arms[sde], SOMATIC__MOTOR_PARAM__MOTOR_POSITION, jointAngles, 7, NULL);
//				
//					sleep(2);
//					hw->updateSensors(1);		
//					somatic_motor_update(&daemon_cx, hw->grippers[sde]);
//
//					wss->resetReferenceTransform();

					// Over
					continue;


				// Reset position
				case 't':
					cout << "Reset arm" << endl;
					cout << "ee: " << wss->endEffector->getWorldTransform().block<3,1>(0,3).transpose() << endl;
					somatic_motor_cmd(&daemon_cx, hw->arms[sde], SOMATIC__MOTOR_PARAM__MOTOR_POSITION, resetAngles, 7, NULL);

					sleep(2);
					hw->updateSensors(1);		
					somatic_motor_update(&daemon_cx, hw->grippers[sde]);

					wss->resetReferenceTransform();
					cout << "ee after reset: " << wss->endEffector->getWorldTransform().block<3,1>(0,3).transpose() << endl;
					continue;

				default:
					cout << "Unknown keyboard input" << endl;
					continue;
		    }

			if (fingerMove)	{
				PrintInfo();
				MoveFingerTo(fingerGoal, closeFinger);
				fingerMove = false;
			} else {
//				MoveEEBy(tomove, FORCEMAGNTRESH);
				MoveWorldBy(tomove, FORCEMAGNTRESH, true);
			}
		}


	usleep(1e5);

	}
}
	
void LogCPDFT(ostream &logfile, bool contact, int direction)
{
	// Contact and direction
	logfile << contact << "  " << direction << "  ";

	// Center of ball position
	Eigen::MatrixXd curr_trans = wss->endEffector->getWorldTransform();
	Eigen::VectorXd curr = curr_trans.block<3,1>(0,3);
	curr[2] = curr[2] + BALLOFFSET;
	logfile << curr.transpose() << "  ";

	// FT
	Eigen::VectorXd forceTorque = hw->fts[sde]->lastExternal;
	logfile << forceTorque.transpose();
	logfile << endl;
}

	
int GetDirection(const Eigen::Vector3d &d)
{
	if (d[0] < -0.01)
		return -1;
	if (d[0] > 0.01)
		return 1;
	if (d[1] < -0.01)
		return -2;
	if (d[1] > 0.01)
		return 2;
	if (d[2] < -0.01)
		return -3;
	if (d[2] > 0.01)
		return 3;
}

	
	

void LogFTP(ostream &logfile)
{	
	// FT 
	Eigen::VectorXd forceTorque = hw->fts[sde]->lastExternal;

	logfile << forceTorque.transpose() << "  ";
	Eigen::Vector3d force = forceTorque.head<3>();
	if (force.norm() > FORCEMAGNTRESH) {
		logfile << "1  ";
		cout << "In contact!" << endl;
	} else {
		logfile << "0  ";
		cout << "Not contact!" << endl;
	}

	
	// ee position
	Eigen::MatrixXd curr_trans = wss->endEffector->getWorldTransform();
	Eigen::VectorXd curr = curr_trans.block<3,1>(0,3);
	logfile << curr.transpose() << endl;
}


void Log(ostream &logfile)
{
	// ee 
	Eigen::MatrixXd curr_trans = wss->endEffector->getWorldTransform();
	Eigen::VectorXd curr = Krang::transformToEuler(curr_trans, math::XYZ);
	logfile << curr.transpose() << endl;

	// FT 
	Eigen::VectorXd forceTorque = hw->fts[sde]->lastExternal;
	logfile << forceTorque.transpose() << endl;

	// finger contact info 
	for (int i = 0; i < 3; i++) {
		logfile << contact_close[i] << '\t';
	}
	logfile << endl;

	for (int i = 0; i < 3; i++) {
		logfile << position_close[i] << '\t';
	}
	logfile << endl;

	for (int i = 0; i < 3; i++) {
		logfile << contact_open[i] << '\t';
	}
	logfile << endl;

	for (int i = 0; i < 3; i++) {
		logfile << position_open[i] << '\t';
	}

	logfile << endl;


}
	






void PrintInfo()
{
	// Print F/T sensor reading
	Eigen::VectorXd forceTorque = hw->fts[sde]->lastExternal;
	cout << "ForceTorque sensor reading: " << forceTorque.transpose() << endl;

	// Print end effector pos and rot
	Eigen::MatrixXd curr_trans = wss->endEffector->getWorldTransform();
	Eigen::VectorXd curr = Krang::transformToEuler(curr_trans, math::XYZ);
	cout << "End effector: " << curr.transpose() << endl;

}


void MoveFingerTo(double const desired_goal[], bool closeFinger)
{

	//// For fingers
	// Current current and position
	double current[] = {0,0,0,0};
	double position[] = {0,0,0,0};

	// Fingers in contact
	bool contact[] = {false, false, false, false};

	// Whether to send a new command because of new finter(s) are in contact
	bool tostop = false;		

	// Whether to output some termination info and finish this move
	bool toterm = true;


	// Desired finger position to achieve
	double goal[4];
	for (int i = 0; i < 4; i++)
		goal[i] = desired_goal[i];

	// Largest current so far
	double largecurrent[] = {0,0,0,0};


	// Start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	double time_accumulated = 0;


	while(!somatic_sig_received) {

		// Update timers
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		time_accumulated += time_delta;

		// Run out of time?
		if (time_accumulated > MAX_CLOSE_TIME) {
			cout << "Finish approaching ABNORMALLY." << endl;
			// Print out current state
			printCurPosCont(current, position, contact);
			break;
		}

		// Update sensors
		hw->updateSensors(time_delta);
		somatic_motor_update(&daemon_cx, hw->grippers[sde]);


		// Read out current and position
		for (int i = 0; i < 4; i++) {
			current[i] = hw->grippers[sde]->cur[i];
			position[i] = hw->grippers[sde]->pos[i];
			if (current[i] > largecurrent[i])
				largecurrent[i] = current[i];
		}


		// Check termination condition
		for (int i = 0; i < 4; i++) {
			// Is this finger still in process?
			if ( (position[i] != goal[i]) && (contact[i] == false)  )
				toterm = false;
		}

		if (toterm) {
			cout << "Finish approaching SUCCESSFULLY. Fingers in contact: ";
			for (int i = 0; i < 4; i++) {
				if (contact[i] == true) {
					cout << i+1 << "   ";
				}
			}
			cout << endl;
			// Print current state
			printCurPosCont(current, position, contact);

			// Print max current
			cout << "Largest current: ";
			for (int i = 0; i < 4; i++)
				cout << largecurrent[i] << ", ";
			cout << endl;

			// Log
			cout << "Log contact info and position" << endl;
			if (closeFinger) {
				for (int i = 0; i < 4; i++) {
					contact_close[i] = contact[i];
					position_close[i] = position[i];
				}
			} else {
				for (int i = 0; i < 4; i++) {
					contact_open[i] = contact[i];
					position_open[i] = position[i];
				}
			}
				



			break;

		} else {
			// Reset toterm
			toterm = true;
		}


		// Check current
		for (int i = 0; i < 4; i++) {

			if ( (current[i] >= current_threshold) && (contact[i] == false) ) {

				cout << "Found new finger in contact: " << i+1 << endl;
				contact[i] = true;
				// Let this finger stop in this position
				tostop = true;
				goal[i] = position[i];
				// Print current state
				printCurPosCont(current, position, contact);

			}
		}

	
		// Reset the goal
		if (tostop) {
			cout << "Reset goal to: ";
			for (int i = 0; i < 4; i++) 
				cout << goal[i] << "   ";
			cout << endl;

			somatic_motor_setpos(&daemon_cx, hw->grippers[sde], goal, 4);
			// Reset tostop
			tostop = false;
		}

		// Print current positions
		for (int i = 0; i < 4; i++) {
//			cout << position[i] << '\t';
		}
//		cout << endl;

		// Print values of current
		for (int i = 0; i < 4; i++) {
			cout << current[i] << '\t';
		}
		cout << endl;
		
	
		// And sleep to fill out the loop period so we don't eat the entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / FINGER_LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
//		usleep(time_sleep_usec);

	}

	
}




void MoveEEBy(const Eigen::VectorXd &change, double maxForce, bool toRecord)
{
	hw->updateSensors(1);		
	somatic_motor_update(&daemon_cx, hw->grippers[sde]);

	wss->resetReferenceTransform();

	Eigen::MatrixXd initial_trans = wss->endEffector->getWorldTransform();
	Eigen::VectorXd initial = Krang::transformToEuler(initial_trans, math::XYZ);
	Eigen::MatrixXd change_trans = Krang::eulerToTransform(change, math::XYZ);
	Eigen::VectorXd end = Krang::transformToEuler(initial_trans * change_trans, math::XYZ);

	cout << "initial: " << initial.transpose() << endl;
	cout << "change: " << change.transpose() << endl;
	cout << "end: " << end.transpose() << endl;

	MoveTo(end, maxForce, toRecord);
}




void MoveWorldBy(const Eigen::VectorXd &change, double maxForce, bool toRecord)
{

	hw->updateSensors(1);		
	somatic_motor_update(&daemon_cx, hw->grippers[sde]);

	wss->resetReferenceTransform();

	Eigen::MatrixXd initial_trans = wss->endEffector->getWorldTransform();
	Eigen::VectorXd initial = Krang::transformToEuler(initial_trans, math::XYZ);
	Eigen::VectorXd end = initial + change;

	cout << "initial: " << initial.transpose() << endl;
	cout << "change: " << change.transpose() << endl;
	cout << "end: " << end.transpose() << endl;

	MoveTo(end, maxForce, toRecord);
}

void MoveTo(const Eigen::VectorXd &end, double maxForce, bool toRecord)
{


	// Start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	double time_accumulated = 0;


	// Other vars
	Krang::Side sde = TOUCHSIDE;
	Eigen::VectorXd zeros6D = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd xdot_approach = Eigen::VectorXd::Zero(6);


	Eigen::MatrixXd initial_trans = wss->endEffector->getWorldTransform();
	Eigen::Vector3d initial_pos = initial_trans.block<3,1>(0,3);
	Eigen::Vector3d move_direction = end.head<3>() - initial_pos;
	int direction = GetDirection(move_direction);
	cout << "Move direction: " << direction << endl;

	if (toRecord)
		LogCPDFT(CPDFTF, false, direction);

	// Move
	while(!somatic_sig_received) {


		// Update timers
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;
		time_accumulated += time_delta;


		// Exceed time limit?
		if (time_accumulated > TIME_LIMIT) {
			setvzero(sde);
			break;
		}


		// Update the robot
		hw->updateSensors(time_delta);
		somatic_motor_update(&daemon_cx, hw->grippers[sde]);

		// Force magnitude exceeds threshold?
		Eigen::VectorXd forceTorque = hw->fts[sde]->lastExternal;
		Eigen::Vector3d force = forceTorque.head<3>();
	
		if (force.norm() > maxForce) {
	
			// Log
			if (toRecord) {
				cout << "Recording CPD" << endl;
				LogCPDFT(CPDFTF, true, direction);
			}

			// Stop approaching
			cout << "Force magnitude: " << force.norm() << " exceeds a threshold, stop approaching" << endl;

			setvzero(sde);
//			somatic_motor_halt(&daemon_cx, hw->arms[Krang::LEFT]);
//			somatic_motor_halt(&daemon_cx, hw->arms[Krang::RIGHT]);
	
			break;
		}
	
		// set up debug printing
		debug_print_this_it = (time_now - time_last_display) > (1.0 / DISPLAY_FREQUENCY);
		if(debug_print_this_it) time_last_display = time_now;
		wss->debug_to_curses = debug_print_this_it;


		if (toRecord && debug_print_this_it) {
			cout << "Recording CPD" << endl;
			LogCPDFT(CPDFTF, false, direction);
		}


		// Achieved goal?
		Eigen::MatrixXd curr_trans = wss->endEffector->getWorldTransform();
//		Eigen::Vector3d curr = curr_trans.block<3,1>(0,3);
		Eigen::VectorXd curr = Krang::transformToEuler(curr_trans, math::XYZ);

		Eigen::VectorXd diff = end- curr;
		Eigen::Vector3d dist_diff = diff.head(3);
		Eigen::Vector3d rad_diff = diff.tail(3);

//		cout << "end: " << end.transpose() << endl;
//		cout << "curr: " << curr.transpose() << endl;

//		cout << "Diff: " << diff.transpose() << endl;
//		cout << "Diff norm: " << diff.norm() << endl;

		if (rad_diff.norm() < RAD_TOLERANCE && dist_diff.norm() < DIST_TOLERANCE) {
			setvzero(sde);

			if (toRecord)
				LogCPDFT(CPDFTF, false, direction);
			cout << "Reached desired position." << endl;
			break;
		}
			

		// Computer xdot
		xdot_approach =  diff /(diff.norm() / SPEEDMAGN);


		// Check for too high currents
		if(Krang::checkCurrentLimits(hw->arms[Krang::LEFT]->cur, 7)
		   && Krang::checkCurrentLimits(hw->arms[Krang::RIGHT]->cur, 7)) {
			cout << "TOO HIGH CURRENTS. HALT MOTORS!" << endl;
			setvzero(sde);
			somatic_motor_halt(&daemon_cx, hw->arms[Krang::LEFT]);
			somatic_motor_halt(&daemon_cx, hw->arms[Krang::RIGHT]);
			break;

			// // TODO: handle this more nicely
			// destroy();
			// exit(EXIT_FAILURE);
		}


		// Jacobian: compute the desired jointspace velocity from the inputs and sensors
		Eigen::VectorXd qdot_jacobian;
		wss->updateFromXdot(xdot_approach, zeros6D,
			nullspace_qdot_refs, time_delta, qdot_jacobian);

		// make sure we're not going too fast
		double magn = qdot_jacobian.norm();
		if (magn > 0.5) qdot_jacobian *= (0.5 / magn);
		if (magn < .05) qdot_jacobian *= 0.0;

		Eigen::VectorXd q = robot->getConfig(*wss->arm_ids);

		// avoid joint limits
		Eigen::VectorXd qdot_avoid(7);
		Krang::computeQdotAvoidLimits(robot, *wss->arm_ids, q, qdot_avoid);

		// add qdots together to get the overall movement
		Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;

		// and apply that to the arm
//		cout << "xdot_approach: " << xdot_approach.transpose() << endl;
//		cout << "qdot_apply: " << qdot_apply.transpose() << endl;

		somatic_motor_setvel(&daemon_cx, hw->arms[sde], qdot_apply.data(), 7);


		if (debug_print_this_it) {
//			Eigen::MatrixXd ee_trans = wss->endEffector->getWorldTransform();
//			Eigen::VectorXd ee_pos = Krang::transformToEuler(ee_trans, math::XYZ);
//			Eigen::Vector3d ee_pos = ee_trans.block<3,1>(0,3);

//			cout << "Accumulated time: " << time_accumulated << endl;
//			cout << "trans" << ee_trans << endl;
//			cout << "current ee pos: " << curr_pos.transpose() << endl;
//			cout << "force norm: " << force.norm() << endl;
//			cout << "xdot_approach: " << xdot_approach.transpose() << endl;
		}

		// And sleep to fill out the loop period so we don't eat the entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		usleep(time_sleep_usec);

	}

}

static void printCurPosCont(double current[], double position[], bool contact[])
{
	// Current
	cout << "Current: ";
	for (int i = 0; i < 4; i++)
		cout << current[i] << ", ";
	cout << endl;

	// Position
	cout << "Position: ";
	for (int i = 0; i < 4; i++)
		cout << position[i] << ", ";
	cout << endl;
	
	// Contact
	cout << "Contact: ";
	for (int i = 0; i < 4; i++)
		cout << contact[i] << ", ";
	cout << endl;

}


/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	cout << "starting..." << endl;
	init();
	run();
	destroy();
}

void setvzero(const Krang::Side sde)
{
	// Set velocity to zero
	// Initialize
	double v[7];
	for (int i = 0; i < 7; i++)
		v[i] = 0;

	// Send command
	somatic_motor_setvel(&daemon_cx, hw->arms[sde], v, 7);
}



static int
tty_unbuffered(int fd)      /* put terminal into a raw mode */
{
    struct termios  buf;

    if (tcgetattr(fd, &buf) < 0)
        return(-1);
        
    save_termios = buf; /* structure copy */

    /* echo off, canonical mode off */
    buf.c_lflag &= ~(ECHO | ICANON);

    /* 1 byte at a time, no timer */
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSAFLUSH, &buf) < 0)
        return(-1);

    ttysavefd = fd;
    return(0);
}

static int
tty_reset(int fd)       /* restore terminal's mode */
{
    if (tcsetattr(fd, TCSAFLUSH, &save_termios) < 0)
        return(-1);
    return(0);
}

static void
tty_atexit(void)        /* can be set up by atexit(tty_atexit) */
{
    if (ttysavefd >= 0)
        tty_reset(ttysavefd);
}

static void
tweak_init()
{
   /* make stdin unbuffered */
    if (tty_unbuffered(STDIN_FILENO) < 0) {
        std::cout << "Set tty unbuffered error" << std::endl;
        exit(1);
    }

    atexit(tty_atexit);

    /* nonblock I/O */
    int flags;
    if ( (flags = fcntl(STDIN_FILENO, F_GETFL, 0)) == 1) {
        perror("fcntl get flag error");
        exit(1);
    }
    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl set flag error");
        exit(1);
    }
}



