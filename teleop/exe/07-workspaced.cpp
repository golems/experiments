/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file 07-workspaced.cpp
 * @author Nehchal Jindal
 * @date Mar 9, 2015
 * @brief It will probably be a workhorse for general tasks, including
 * demonstrations. Moves the Krang robot grippers with workspace control
 *
 * Supports:
 *    . Synch mode where relative transforms b/w two grippers remains same
 *    . Velocity control and pose control.
 * 
 * On chaning input mode, any previous data on the ACH channels is discarded.
 * VEL -> POSE: Current position is set at reference position till any new
				pose cmd is received.
 * POSE -> VEL: Target velocity is set to zero till any vel command is received.
 *
 * Channel Specifications:
 *    Subscribes to:
 *      Waypoints channel: Matrix of size N x 6, where N is number of waypoints.
 *      Ref Vel Channel:   Matrix of size 6 x 1 or 1 x 6
 *      Cmd channel:       Matrix of size 1 x 2.
 *    Publishes to:
 *      Left gripper pose:  Matrix of size 1 x 6
 *      Right gripper pose: Matrix of size 1 x 6
 *      Body state:         Matrix of size 1 x 24  
 *  
 *  NOTE: There should be minimum delay of 100ms between two successive ACH 
 *        message. This is especially important after changing cmd msg is sent
 *        for changing input mode.        
 *
 *
 * The program runs in two threads:
 *      kbhit - This takes input from key-board and takes appropriate action
 *      run - everything else including receiving from and publishing to ACH 
 *            channels, running pose/vel control, printing to screen, etc.
 *      
 *      Some variables are written by both threads, so mutex lock has been 
 *      used.
 * 
 * Default Settings
		. Compliance Mode - ON
		. Input Mode - Pose
 *
 *  TODOs
		. Display current values on the screen
		. Search for TODO in this file. Some tasks may be mentioned int the comments/
 *
 */

#include <kore.hpp>
#include <kore/workspace.hpp>
#include <kore/display.hpp>
#include <kore/util.hpp>
#include <somatic/msg.h>

#include <trajectory_io.h>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>

#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <iostream>
#include <stdio.h>

#include <argp.h>
#include <ncurses.h>
 

using namespace std;

/* Configuration */
#define MAX_NUM_WAYPTS  10

const char *g_default_cmd_chan_name = "cmd_workspaced";

/* synch mode means when relative transforms between the two hands remains the 
   same. When it is on, hand designated as off follows the primary hand */
#define SYNCH_OFF       0      //< synch mode is off
#define SYNCH_ON_L_PRIM 1      //< synch on with left hand primary
#define SYNCH_ON_R_PRIM 2      //< synch on with right hand primary

const char* synch_mode_to_string(int mode){
	switch(mode) {
	case SYNCH_OFF: return "SYNCH_OFF";
	case SYNCH_ON_L_PRIM: return "SYNCH_ON_L_PRIM";
	case SYNCH_ON_R_PRIM: return "SYNCH_ON_R_PRIM";
	}
	return "UNKNOWN";
}

#define IMU_OFF 0          //< don't call kore::updateSensors each timestep
#define IMU_ON 1           //< call kore::updateSensors each timestep

const char* bool_to_stringOnOff(int mode){ 
    if (mode) return "ON"; 
    return "OFF"; 
}

/* In vel mode, gripper aims for target velocity. In pose mode, grippper
 * aims for target pose */
#define INPUT_MODE_VEL  0
#define INPUT_MODE_POSE 1

const char* input_mode_to_string(int mode){
	return mode == INPUT_MODE_VEL ?
				"VELOCITY" : (mode == INPUT_MODE_POSE ? "POSE" : "UNKNOWN");
}

#define COMPLIANCE_OFF 0
#define COMPLIANCE_ON 1   

const char* compliance_mode_to_string(int mode){
	switch(mode) {
	case COMPLIANCE_OFF: return "COMPLIANCE_OFF";
	case COMPLIANCE_ON: return "COMPLIANCE_ON";
	}
	return "UNKNOWN";
}

#define L_GRIP   0     // left gripper
#define R_GRIP   1     // right gripper

/* Prints a Eigen row or column vector. */
#define PRINT_VECTOR(X)                                   \
			printw("(%d x %d) ", (X).rows(), (X).cols()); \
			printw("[");                                  \
			for(int i = 0; i < (X).size(); i++){          \
				printw(((X)[i]) >= 0 ? "  " : " ");       \
				printw("%.4f", (X)[i]);                   \
			}                                             \
			printw("]\n\r");

/* Prints a array of floats */
#define PRINT_ARRAY_FLOAT(X, N)                           \
			printw("[");                                  \
			for(int i = 0; i < N; i++){                   \
				printw(((X)[i]) >= 0 ? "  " : " ");       \
				printw("%.4f", (X)[i]);                   \
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

// context struct
typedef struct {
	somatic_d_t d;
	somatic_d_opts_t d_opts;
	
	// set by command line arguments
	const char *lgrip_waypnts_channel_name;	// left gripper waypoints channel name
	const char *rgrip_waypnts_channel_name;	// right gripper waypoints channel name
	const char *lgrip_pose_channel_name;
	const char *rgrip_pose_channel_name;
	const char *lpincher_state_channel_name;
	const char *rpincher_state_channel_name;
	int opt_verbosity;

	// channels to publish
	ach_channel_t channel_ft[2]; // force and torque data
	ach_channel_t channel_body_state;

	// channels to subscribe
	ach_channel_t channel_ref_vel[2]; // to read ref vel in vel mode
	ach_channel_t channel_cmd;

	std::string init_wd; // The initial working directory

	// set by key-bindings when daemon is running
	int synch_mode;         //< 3-modes off, on-left primary, on-right primary
	bool send_motor_cmds;   //< boolean on or off
	int input_mode;         //< vel or pose
	int imu_mode;           //< int 0=off, 1=on (not bool b/c we might add more options)
	int compliance_mode;    //< int: COMPLIANCE_OFF, COMPLIANCE_ON
	bool show_key_bindings; //< if true, show key bindings in ncurses display

	Eigen::VectorXd ref_poses[2];   // for pose input mode
	Eigen::VectorXd ref_vel[2];     // for vel input mode

	// store the list of waypoints. First element is the first element.
	vector<Eigen::VectorXd> refTraj[2];
	int numWayPtsReached;

	Eigen::MatrixXd T_off_to_pri;

	// max currents till now
	float maxCurrent[2][7];

} cx_t;

/* GLOBAL VARIABLES (Ideally, global variables should be as minimum as 
   possible) */
cx_t cx;

// TODO: move the channels into the context struct
// channels for for reading waypoints
ach_channel_t waypts_channel[2]; 

// TODO: move the channels into the context struct
// channels for publishing state
ach_channel_t lgrip_pose_channel, rgrip_pose_channel, lpincher_state_channel, rpincher_state_channel; 

// options
static struct argp_option options[] = {
	{
		.name = "verbose",
		.key = 'v',
		.arg = NULL,
		.flags = 0,
		.doc = "Causes verbose output"
	},
	{
		.name = "lgrip-waypnts",
		.key = 'l',
		.arg = "CHANNEL-NAME",
		.flags = 0,
		.doc = "ach channel name for reading left gripper waypoints in world frame"
	},
	{
		.name = "rgrip-waypnts",
		.key = 'r',
		.arg = "CHANNEL-NAME",
		.flags = 0,
		.doc = "ach channel name for reading right gripper waypoints in world frame"
	},
	{
		.name = "lgrip-pose",
		.key = 'a',
		.arg = "CHANNEL-NAME",
		.flags = 0,
		.doc = "ach channel name to publish left gripper pose in world frame"
	},
	{
		.name = "rgrip-pose",
		.key = 'b',
		.arg = "CHANNEL-NAME",
		.flags = 0,
		.doc = "ach channel name to publish right gripper pose in world frame"
	},
	{
		.name = NULL,
		.key = 0,
		.arg = NULL,
		.flags = 0,
		.doc = NULL
	}
};

/// parser (fxn to parse single command line argument)
static int parse_opt( int key, char *arg, struct argp_state *state) {
	cx_t *cx = (cx_t*)state->input;
	switch(key) {
	case 'v':
		cx->opt_verbosity++;
		break;
	case 'l':
		cx->lgrip_waypnts_channel_name = strdup(arg);
		break;
	case 'r':
		cx->rgrip_waypnts_channel_name = strdup(arg);
		break;
	case 'a':
		cx->lgrip_pose_channel_name = strdup(arg);
		break;
	case 'b':
		cx->rgrip_pose_channel_name = strdup(arg);
		break;
	case 'c':
		cx->lpincher_state_channel_name = strdup(arg);
		break;
	case 'd':
		cx->rpincher_state_channel_name = strdup(arg);
		break;
	case 0:
		break;
	}
	
	somatic_d_argp_parse( key, arg, &cx->d_opts );
	return 0;
}

/// argp program version
const char *argp_program_version = "Workspace daemon 1.0.0";
/// argp program bug address
const char *argp_program_bug_address = "nehchal@gatech.edu";
/// argp program arguments documentation
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "Moves the Krang robot grippers with workspace control";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };

/* Find implementation later in this file */
void recordEvent(const char* event);
void printEvents();

void parse_args(int argc, char* argv[])
{
	memset(&cx, 0, sizeof(cx));

	// default options
	cx.lgrip_waypnts_channel_name = "krang_lgrip_waypts";
	cx.rgrip_waypnts_channel_name = "krang_rgrip_waypts";
	cx.lgrip_pose_channel_name = "krang_lgrip_pose";
	cx.rgrip_pose_channel_name = "krang_rgrip_pose";
	cx.lpincher_state_channel_name = "krang_lpincher_state";
	cx.rpincher_state_channel_name = "krang_rpincher_state";
	cx.d_opts.ident = "workspaced";
	cx.d_opts.sched_rt = SOMATIC_D_SCHED_UI;
	cx.opt_verbosity = 0;

	argp_parse(&argp, argc, argv, 0, NULL, &cx);

	// initialize as somatic daemon
	somatic_d_init(&cx.d, &cx.d_opts);

	if( cx.opt_verbosity ) {
		fprintf(stderr, "\n* workspaced *\n");
		fprintf(stderr, "Verbosity:  %d\n", cx.opt_verbosity);
		fprintf(stderr, "Left gripper waypoints ACH channel:  %s\n", cx.lgrip_waypnts_channel_name);
		fprintf(stderr, "Right gripper waypoints ACH channel: %s\n", cx.rgrip_waypnts_channel_name);
		fprintf(stderr,"-------\n");
   }
}

// initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.5; //0.1;
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.50; // maximum 50 cm per second from spacenav
const double SPACENAV_TRANSLATION_GAIN = 0.25; // maximum .25 radians per second from spacenav
const double COMPLIANCE_TRANSLATION_GAIN = 0.001; // 0.002;
const double COMPLIANCE_ORIENTATION_GAIN = 0.01; // 0.10;
const double HAND_OVER_HAND_SPEED = 0.05; // 3 cm per second when going hand-over-hand

// various rate limits - be nice to other programs
const double LOOP_FREQUENCY = 15.0;
const double DISPLAY_FREQUENCY = 3.0;

// display information
const int CURSES_DEBUG_DISPLAY_START = 20;

// gripper reference messages, so we don't have to do initialization in the middle of our program
double GRIPPER_CURRENT_OPEN[] = {10.0};
double GRIPPER_CURRENT_CLOSE[] = {-10.0};
double GRIPPER_CURRENT_ZERO[] = {0.0};

// Gripper constants (could move to constructor, but convenient here)
double SCHUNK_GRIPPER_POSITION_OPEN[] = {0.9};
double SCHUNK_GRIPPER_POSITION_CLOSE[] = {0};
double SCHUNK_GRIPPER_POSITION_PARTIAL[] = {0.4};

/* ********************************************************************************************* */
// State variables

// hardware objects
Krang::Hardware* hw;                            ///< connects to hardware
// bool sending_commands = false;
std::map<Krang::Side, Krang::SpaceNav*> spnavs; ///< points to spacenavs

// process state
somatic_d_t daemon_cx;                          ///< daemon context

// pointers to important DART objects
simulation::World* world;
dynamics::SkeletonDynamics* robot;

// workspace stuff
std::map<Krang::Side, Krang::WorkspaceControl*> wss; ///< does workspace control for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_q_refs; ///< nullspace configurations for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_q_masks; ///< nullspace configurations for the arms
std::map<Krang::Side, Krang::Vector7d> nullspace_qdot_refs; ///< nullspace configurations for the arms
Eigen::MatrixXd Trel_pri_to_off; ///< translation from the primary hand to the off hand

Krang::Side primary_hand = Krang::LEFT;
Krang::Side off_hand = Krang::RIGHT;

// debug information
bool debug_print_this_it;       ///< whether we print

/// Clean up
void destroy() {

	Krang::destroy_curses();   // close display

	// Stop the daemon
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
					SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
	
	// close the channel we use for publishing visualization data
	// somatic_d_channel_close(&daemon_cx, &vis_chan);

	// Clean up the workspace stuff
	delete wss[Krang::LEFT];
	delete wss[Krang::RIGHT];

	// Close down the hardware
	delete spnavs[Krang::LEFT];
	delete spnavs[Krang::RIGHT];
	delete hw;

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* Prints the keyboard keys and \corresponding actions to the console */
void print_key_bindings(){
	printw("\t\t[KEY BINDINGS]");
	if(!cx.show_key_bindings)
		{printw("    (Press 'b' to show)\n"); return;}

	printw("(Press 'b' to show)\n%s", 
	 "'m','M': toggle ON and OFF sending commands to motors                     \n"
	 "'z'    : Rotates among 3 synch mode. 'OFF', 'ON - left primary',          \n"
	 "          'ON - right primary' . If synch mode is ON, then trajectory on  \n"
	 "          waypoints channel for off hand is ignored.                      \n"
	 "'c'    : toggle COMPLIANCE mode (ON or OFF)                               \n"
	 "'i'    : toggle IMU mode (ON or OFF)                                      \n"
	 "'g'    : Re-read and update compliance gains from file                    \n"
	 "          (workspaced-compliance-gains.txt)                               \n"
	 "'q'    : Quit                                                             \n"
	 "'r'    : Reset Motors                                                     \n"
	 "'p'    : Toggle input mode between VELOCITY and POSE control              \n"
	 "'1'    : Close left gripper                                               \n"
	 "'2'    : Open left gripper                                                \n"
	 "'3'    : Close right gripper                                              \n"
	 "'4'    : Open right gripper                                               \n");
	return;
}

/* Gets current pose of the gripper in World frame
 * @param gripperBodyNode : The pointer to the gripper node in the DART model
 * Returns the 6-vector pose of gripper in World frame */
Eigen::VectorXd getCurPose(kinematics::BodyNode *gripperBodyNode){
	return Krang::transformToEuler(gripperBodyNode->getWorldTransform(), math::XYZ);
}

/* Gets current state of the pincher (position and velocity)
 * @param gripper : The pointer to the pincher motor message
 * Returns the 2-vector [pos, vel] containing the pincher state */
Eigen::VectorXd getCurPincherState(somatic_motor_t* pincher)
{
	Eigen::VectorXd gripper_state(2);
	gripper_state << pincher->pos[0], pincher->vel[0];
	return gripper_state;
}

/* Set the input mode (pose or velocity). Uses global variable cx.
 * inputMode: INPUT_MODE_POSE or INPUT_MODE_VEL */
void setInputMode(int inputMode){
	if ((cx.input_mode = inputMode) == INPUT_MODE_VEL){
		ach_flush(&cx.channel_ref_vel[L_GRIP]);
		ach_flush(&cx.channel_ref_vel[R_GRIP]);

		cx.ref_vel[L_GRIP] = Eigen::VectorXd::Zero(6);
		cx.ref_vel[R_GRIP] = Eigen::VectorXd::Zero(6);
	}
	else{   // pose mode
		ach_flush(&waypts_channel[L_GRIP]);
		ach_flush(&waypts_channel[R_GRIP]);

		cx.ref_poses[L_GRIP] = getCurPose(wss[Krang::LEFT]->endEffector);
		cx.ref_poses[R_GRIP] = getCurPose(wss[Krang::RIGHT]->endEffector);

		cx.refTraj[L_GRIP].clear();
		cx.refTraj[R_GRIP].clear();
	}
	return;
}

/* Open the gripper. Uses global var daemon_cx.*/
void openGripper(somatic_motor_t *gripper){
	somatic_motor_reset(&daemon_cx, gripper);
	usleep(1e4);
	somatic_motor_setpos(&daemon_cx, gripper, SCHUNK_GRIPPER_POSITION_OPEN, 1);
	return;
}

/* Close the gripper. Uses global var daemon_cx. */
void closeGripper(somatic_motor_t *gripper){
	somatic_motor_reset(&daemon_cx, gripper);
	usleep(1e4);
	somatic_motor_setpos(&daemon_cx, gripper, SCHUNK_GRIPPER_POSITION_CLOSE, 1);
	return;
}

/* [This variable should be put into cx_t. But if this is put up there, 
	the program throws seg fault during execution] */
std::string init_wd;
/* Outputs the complaince gains from the file. Use global variable cx */
void readComplianceGains(double* translationGain, double* orientationGain) {
	std::string gains_file = init_wd + "/../data/workspaced-compliance-gains.txt";
	ifstream file(gains_file.c_str());

	assert(file.is_open());
	char line [1024];
	file.getline(line, 1024);
	while (line[0] == '#')      // Ignore lines starting with hash
		file.getline(line, 1024);
	std::stringstream stream(line, std::stringstream::in);
	size_t i = 0;
	// double newDouble;
	// while ((i < 8) && (stream >> newDouble)) K(i++) = newDouble;
	stream >> *translationGain >> *orientationGain;
	
	file.close();
	return;
}

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
void *kbhit(void *) {
    char ch;

    // general curses stuff
    initscr();
    clear();
    noecho();                   // do not echo input to the screen
    cbreak();                   // do not buffer by line (receive characters immediately)
    timeout(0);                 // non-blocking getch

    while(true){ 
        ch = getch(); 
        pthread_mutex_lock(&mutex);
        switch (ch) {
        case 'q': somatic_sig_received = true; break;
        case 'r': {
            somatic_motor_reset(&daemon_cx, hw->arms[Krang::LEFT]);
            somatic_motor_reset(&daemon_cx, hw->arms[Krang::RIGHT]);
        } break;
        case 'h': {
            cx.send_motor_cmds = false;
            somatic_motor_halt(&daemon_cx, hw->arms[Krang::LEFT]);
            somatic_motor_halt(&daemon_cx, hw->arms[Krang::RIGHT]);
        } break;              
        case 'p':
            setInputMode(!cx.input_mode);
            break;
        case 'z': {
            // change synch mode
            Krang::Side primary, off;
            if(cx.synch_mode == SYNCH_OFF){
                cx.synch_mode = SYNCH_ON_L_PRIM;
                primary = Krang::LEFT;
                off = Krang::RIGHT;
            }
            else if(cx.synch_mode == SYNCH_ON_L_PRIM){
                cx.synch_mode = SYNCH_ON_R_PRIM;
                primary = Krang::RIGHT;
                off = Krang::LEFT;
            }
            else
                cx.synch_mode = SYNCH_OFF;

            Eigen::Matrix4d T_pri_to_world = wss[primary]->endEffector->getWorldTransform();
            Eigen::Matrix4d T_off_to_world = wss[off]->endEffector->getWorldTransform();

            cx.T_off_to_pri = T_pri_to_world.inverse() * T_off_to_world;
        } break;
		case 'b': {
		cx.show_key_bindings = !cx.show_key_bindings;
			if (!cx.show_key_bindings)
				clear(); // clears extra space from display
				break;
		}
        case 'c': // toggle compliance mode
            if(cx.compliance_mode == COMPLIANCE_ON){
                wss[Krang::LEFT]->setComplianceOff();
                wss[Krang::RIGHT]->setComplianceOff();
                cx.compliance_mode = COMPLIANCE_OFF;
            }
            else{ // turn compliance on
                wss[Krang::LEFT]->setComplianceOn();
                wss[Krang::RIGHT]->setComplianceOn();
                cx.compliance_mode = COMPLIANCE_ON;
            }
            break;            
        case 'i':
            // change sensor mode
            if(cx.imu_mode = !cx.imu_mode)
                hw->setImuOn();
            else
                hw->setImuOff();
            break;
        case 'g': // update the gains
            double tGain, oGain;
            readComplianceGains(&tGain, &oGain);
            wss[Krang::LEFT]->updateComplianceGains(tGain, oGain);
            wss[Krang::RIGHT]->updateComplianceGains(tGain, oGain);
            break;
        case 'm': case 'M': case ' ':
            // toggle sending commands to motors
            cx.send_motor_cmds = !cx.send_motor_cmds;

            if (cx.send_motor_cmds) {
                somatic_motor_reset(&daemon_cx, hw->arms[Krang::LEFT]);
                somatic_motor_reset(&daemon_cx, hw->arms[Krang::RIGHT]);
            } else {
                Eigen::VectorXd z = Eigen::VectorXd::Zero(7);
                somatic_motor_setvel(&daemon_cx, hw->arms[Krang::LEFT], z.data(), 7);
                somatic_motor_setvel(&daemon_cx, hw->arms[Krang::RIGHT], z.data(), 7);
            }
            break;
        case '1':   // experimental functionality for developer. 
            closeGripper(hw->grippers[Krang::LEFT]);
            break;
        case '2': 
            openGripper(hw->grippers[Krang::LEFT]);
            break;
        case '3': 
            closeGripper(hw->grippers[Krang::RIGHT]);
            break;
        case '4': 
            openGripper(hw->grippers[Krang::RIGHT]);
            break;
        case '9': // debugging functionality
        	cx.send_motor_cmds = false;
			somatic_motor_halt(&daemon_cx, hw->arms[Krang::LEFT]);
        	break;
        default:
            //cout<<__LINE__<<": Value of ch is :" << (int)ch <<std::endl<<'\r';
            break;
        }

        pthread_mutex_unlock(&mutex);
    }
}

/* Prints the current status of the machine */
void print_robot_state(const Krang::Hardware* hw,
					   const Krang::WorkspaceControl *l_ws, 
					   const Krang::WorkspaceControl *r_ws, 
					   const cx_t& cx){

    printw("Compliance lin/ang gains: %f %f\n", 
        l_ws->compliance_translation_gain,
        l_ws->compliance_orientation_gain);
    printw("CURRENT CONF. (use key-bindings to change)\n\r");
    printw("   Synch Mode: %s\n\r", synch_mode_to_string(cx.synch_mode));
    printw("   Send commands to Motor:");
    printw(cx.send_motor_cmds ? "ON\n" : "OFF\n");
    printw("    Input Mode: %s\n\r", input_mode_to_string(cx.input_mode));
    printw("    Sensor (IMU) update Mode: IMU_%s\n\r", bool_to_stringOnOff(cx.imu_mode));
    printw("    Compliance Mode: %s\n\r", compliance_mode_to_string(cx.compliance_mode));
    printw("----------------------------------\n");

	Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd pincher_state = Eigen::VectorXd::Zero(2);

	printw("ROBOT STATE\n\r");

	// TODO the pose used exponential coordinates. Covert it to euler
	printw("Base pose (in World Frame):\n");
	pose = hw->robot->getConfig(Krang::dart_root_dof_ids);
	printw("\texp:   ");
	PRINT_VECTOR(pose)
	printw("\teuler: ");
	kinematics::BodyNode* baseBodyNode = hw->robot->getNode("Base");
	pose = Krang::transformToEuler(baseBodyNode->getWorldTransform(), math::XYZ);
	PRINT_VECTOR(pose)

	printw("\n[Left Gripper]\n");

	printw(" Current Pose (in Robot Frame):\n     ");
	pose = getCurPose(l_ws->endEffector);
	PRINT_VECTOR(pose)

	printw(" Ref Pose (in Robot Frame):\n     ");
	PRINT_VECTOR(cx.ref_poses[L_GRIP])

	printw(" Ref Vel (in Robot Frame):\n     ");
	PRINT_VECTOR(cx.ref_vel[L_GRIP])

	printw(" Joint Angles:\n\r    ");
	pose = hw->robot->getConfig(*wss[Krang::LEFT]->arm_ids);
	PRINT_VECTOR(pose)

	printw(" Pincher state [pos, vel]:\n\r    ");
	pincher_state = getCurPincherState(hw->grippers[Krang::LEFT]);
	PRINT_VECTOR(pincher_state)

	printw("\n[Right Gripper]\n");

	printw(" Current Pose (in Robot Frame):\n     ");
	pose = getCurPose(r_ws->endEffector);
	PRINT_VECTOR(pose)

	printw(" Ref Pose (in Robot Frame):\n    ");
	PRINT_VECTOR(cx.ref_poses[R_GRIP])

	printw(" Ref Vel (in Robot Frame):\n\r     ");
	PRINT_VECTOR(cx.ref_vel[R_GRIP])

	printw(" Joint Angles:\n\r    ");
	pose = hw->robot->getConfig(*wss[Krang::RIGHT]->arm_ids);
	PRINT_VECTOR(pose)

	printw(" Pincher state [pos, vel]:\n\r    ");
	pincher_state = getCurPincherState(hw->grippers[Krang::RIGHT]);
	PRINT_VECTOR(pincher_state)

	printw("\n");

	printw("Force/Torque values [Left Gripper]\n    ");
	PRINT_VECTOR(hw->fts[Krang::LEFT]->lastExternal)

	printw("Force/Torque values [Right Gripper]\n    ");
	PRINT_VECTOR(hw->fts[Krang::RIGHT]->lastExternal)

	printw("Krang Body Config\n    ");
	pose = hw->robot->getPose();
	PRINT_VECTOR(pose)

	printw("Current (Amps) [Left]: ");
	PRINT_ARRAY_FLOAT(hw->arms[Krang::LEFT]->cur, 7)
	printw("Max (Amps) [Left]: ");
	PRINT_ARRAY_FLOAT(cx.maxCurrent[L_GRIP], 7)
	printw("Current (Amps) [Right]: ");
	PRINT_ARRAY_FLOAT(hw->arms[Krang::RIGHT]->cur, 7)
	printw("Max (Amps) [Right]: ");
	PRINT_ARRAY_FLOAT(cx.maxCurrent[R_GRIP], 7)

	printw("\nNum of Waypoints reached: %d\n", cx.numWayPtsReached);

	return;
}

/* Publishes eigen matrix to the given channel. This handles serialization and
 * error check while putting on the channel. */
void publish_vector_to_channel(ach_channel_t& chan, Eigen::VectorXd& vec){

	char buf[2 * sizeof(int) + vec.size() * sizeof(double)];
	serialize_from_Matrix(vec.transpose(), buf);

	assert(ACH_OK== ach_put( &chan, buf, sizeof(buf)));
	return;
}

/* Prints the current status of the machine */
void publish_to_channels(const Krang::Hardware* hw,
					   const Krang::WorkspaceControl *l_ws, 
					   const Krang::WorkspaceControl *r_ws, 
					   cx_t& cx){
	Eigen::VectorXd pose;
	Eigen::VectorXd pincher_state;

	pose = getCurPose(l_ws->endEffector);
	publish_vector_to_channel(lgrip_pose_channel, pose);

	pose = getCurPose(r_ws->endEffector);
	publish_vector_to_channel(rgrip_pose_channel, pose);

	pincher_state = getCurPincherState(hw->grippers[Krang::LEFT]);
	publish_vector_to_channel(lpincher_state_channel, pincher_state);

	pincher_state = getCurPincherState(hw->grippers[Krang::RIGHT]);
	publish_vector_to_channel(rpincher_state_channel, pincher_state);

	pose = hw->robot->getPose();
	// remove first 6 number (pose of root node)
	// pose = pose.tail(pose.size() - 6);  
	publish_vector_to_channel(cx.channel_body_state, pose);

	pose = hw->fts[Krang::LEFT]->lastExternal;
	publish_vector_to_channel(cx.channel_ft[L_GRIP], pose);

	pose = hw->fts[Krang::RIGHT]->lastExternal;
	publish_vector_to_channel(cx.channel_ft[R_GRIP], pose);

	return;
}

/**
 * Reads the next waypoint from the given ACH channel.
 * 
 * @param chan : [IN/OUT] the ACH channel to read from.
 * @param waypnt: [OUT] the read data is written to this. 
 * Returns true if next waypoints read successfully from the channel, false
 *  otherwise. */
bool poll_waypnts_channel(ach_channel_t& chan, 
									std::vector<Eigen::VectorXd>& refTraj) { 
	size_t frame_size = 0;
	
	char buf[2 * sizeof(int) + MAX_NUM_WAYPTS * 6 * sizeof(double)];

	enum ach_status r = ach_get(&chan, buf, sizeof(buf), &frame_size, NULL, ACH_O_LAST);
	if(!(r == ACH_OK || r == ACH_MISSED_FRAME))
		return false;

	Eigen::MatrixXd refTrajM = deserialize_to_Matrix(buf);
	if(refTrajM.cols() != 6){
		printw("Error at Line %d: Length of way-point is not 6\n\r", __LINE__);
		return false;
	}
	
	refTraj.clear();

	Eigen::VectorXd temp;
	for(int i=0; i < refTrajM.rows(); i++){
		temp = refTrajM.row(i);
		refTraj.push_back(temp);
	}
	return true;
}

/**
 * Reads the next velocity from the given ACH channel.
 * 
 * @param chan : [IN/OUT] the ACH channel to read from.
 * @param vel: [OUT] the read data is written to this. 
 * Returns true if next waypoints read successfully from the channel, false
 *  otherwise. */
bool poll_ref_vel_channel(ach_channel_t& chan, Eigen::VectorXd& vel) { 
	size_t frame_size = 0;  
	char buf[2 * sizeof(int) + 6 * sizeof(double)];

	enum ach_status r = ach_get(&chan, buf, sizeof(buf), &frame_size, NULL, ACH_O_LAST);
	if(!(r == ACH_OK || r == ACH_MISSED_FRAME))
		return false;

	Eigen::MatrixXd velM = deserialize_to_Matrix(buf);
	if(velM.size() != 6){
		printw("Error at Line %d: Length of way-point is not 6\n\r", __LINE__);
		return false;
	}

	//vel = ( (velM.rows() == 6) ? velM.col(0) : velM.row(0) ) ;
	if(velM.rows() == 6)
		vel = velM.col(0);
	else
		vel = velM.row(0);
	
	return true;
}

/* Reads the command channel and takes action. Makes changes to global 
 * variable cx.
 *  chan :[IN] the channel to read the command from */
void poll_cmd_channel(ach_channel_t& chan){

    size_t frame_size = 0;  
    char buf[2 * sizeof(int) + 2 * sizeof(double)];

    enum ach_status r = ach_get(&chan, buf, sizeof(buf), &frame_size, NULL, ACH_O_LAST);
    if(!(r == ACH_OK || r == ACH_MISSED_FRAME)) return;

    // printw("\n%d:I am inside %s()\n", __LINE__, __func__);

    Eigen::MatrixXd cmdM = deserialize_to_Matrix(buf);
    if(cmdM.size() != 2)
        printw("Error at Line %d: Invalid message on command code\n\r", __LINE__);

    // printw("\n%d:I am inside %s()\n", __LINE__, __func__);

    int cmdCode = int(cmdM(0) + 0.5);   // round the value
    int val = int(cmdM(1) + 0.5);       // round the value

    switch(cmdCode){ // round a command code
        case 0: 
            (val == 0) ? 
                setInputMode(INPUT_MODE_POSE)
                    : setInputMode(INPUT_MODE_VEL);
        case 1:     // close/open both grippers
            if(val == 0){
                openGripper(hw->grippers[Krang::LEFT]);
                openGripper(hw->grippers[Krang::RIGHT]);
            }
            else{
                closeGripper(hw->grippers[Krang::LEFT]);
                closeGripper(hw->grippers[Krang::RIGHT]);
            }
            break;
        case 2: 
            if(val == 0)
                openGripper(hw->grippers[Krang::LEFT]);
            else
                closeGripper(hw->grippers[Krang::LEFT]);
            break;
        case 3: break;
            if(val == 0)
                openGripper(hw->grippers[Krang::RIGHT]);
            else
                closeGripper(hw->grippers[Krang::RIGHT]);
            break;
        case 4: 
            if(val) hw->setImuOn();
            else hw->setImuOff();
            break;
        default: 
            break;
    }

    printw("\nReceived cmd. cmd = %d, val = %d\n", cmdCode, val);
    return;
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());

	// set the initial target pose to same as current pose
	// Pose of Gripper in World Frame
	Eigen::Matrix4d T_gripper_to_world;
	T_gripper_to_world =  wss[Krang::LEFT]->endEffector->getWorldTransform();
	cx.ref_poses[L_GRIP] = Krang::transformToEuler(T_gripper_to_world, math::XYZ);

	T_gripper_to_world = wss[Krang::RIGHT]->endEffector->getWorldTransform();
	cx.ref_poses[R_GRIP] = Krang::transformToEuler(T_gripper_to_world, math::XYZ);

	cx.ref_vel[L_GRIP] = Eigen::VectorXd::Zero(6);
	cx.ref_vel[R_GRIP] = Eigen::VectorXd::Zero(6);

	Eigen::VectorXd x; // current position of the gripper

	while(!somatic_sig_received) {

		pthread_mutex_lock(&mutex);     

		Krang::curses_display_row = CURSES_DEBUG_DISPLAY_START;

		// ========================================================================================
		// Update state: get passed time and kinematics, sensor input and check for current limit

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// set up debug printing
		debug_print_this_it = (time_now - time_last_display) > (1.0 / DISPLAY_FREQUENCY);
		if(debug_print_this_it) time_last_display = time_now;
		wss[Krang::LEFT]->debug_to_curses = debug_print_this_it;
		wss[Krang::RIGHT]->debug_to_curses = debug_print_this_it;

		/* Read the robot state from sensors. This includes reading joint 
		   angles for arms. It also update internal kinematics. */
		hw->updateSensors(time_delta);

		// Check for too high currents
		if(Krang::checkCurrentLimits(hw->arms[Krang::LEFT]->cur, 7)
		   && Krang::checkCurrentLimits(hw->arms[Krang::RIGHT]->cur, 7)) {
			if (cx.send_motor_cmds) {
				cx.send_motor_cmds = false;
				somatic_motor_halt(&daemon_cx, hw->arms[Krang::LEFT]);
				somatic_motor_halt(&daemon_cx, hw->arms[Krang::RIGHT]);
			}

			// // TODO: handle this more nicely
			// destroy();
			// exit(EXIT_FAILURE);
		}

		// update max currents till now
		for (int i=0; i<7; i++){
			cx.maxCurrent[L_GRIP][i] = fmax(cx.maxCurrent[L_GRIP][i], abs(hw->arms[Krang::LEFT]->cur[i]));
			cx.maxCurrent[R_GRIP][i] = fmax(cx.maxCurrent[R_GRIP][i], abs(hw->arms[Krang::RIGHT]->cur[i]));
		}

		// ========================================================================================
		// Perform workspace for each arm, changing the input for right arm based on synch mode

		for(int side = Krang::LEFT; side < 2; side++) {  // L_GRIP and L_RIGHT
			// Get the arm side and the joint angles
			Krang::Side sde = static_cast<Krang::Side>(side);
			Krang::Side sde_other = (sde==Krang::LEFT) ? Krang::RIGHT : Krang::LEFT;
			Eigen::VectorXd q = robot->getConfig(*wss[sde]->arm_ids);

			/* Get current pose of the gripper */
			x = getCurPose(wss[sde]->endEffector);

			Eigen::VectorXd qdot_jacobian;

			// Nullspace: construct a qdot that the jacobian will bias toward using the nullspace
			nullspace_qdot_refs[sde] = (nullspace_q_refs[sde] - q).cwiseProduct(nullspace_q_masks[sde]);

			if(cx.input_mode == INPUT_MODE_VEL) {
				// write here code
				Eigen::VectorXd xDot = cx.ref_vel[side];

				// if (synch mode on) and (it is off hand) then follow primary hand
				if ((cx.synch_mode == SYNCH_ON_L_PRIM && sde == Krang::RIGHT) ||
					  (cx.synch_mode == SYNCH_ON_R_PRIM && sde == Krang::LEFT))
						xDot = cx.ref_vel[1-side];

				
				/* passing time argument as 1.
				  We need to get qdot from xdot using jacobian and time is not 
				  required. But kore seems to be inefficient. Reseting the 
				  reference transform passing time as 1 is just a hacky way to
				  do, till kore is upgraded (See workspace.c in kore to 
				  understand what's happening). */
				wss[sde]->resetReferenceTransform();
				wss[sde]->updateFromXdot(xDot, hw->fts[sde]->lastExternal,
										  nullspace_qdot_refs[sde], 1, qdot_jacobian);
			}

			else { // Pose mode. Apply K_p and get velocity in task space              
				Eigen::MatrixXd T_ref;
				T_ref = Krang::eulerToTransform(cx.ref_poses[side], math::XYZ);

				
				// if (synch mode on) and (it is off hand) then follow primary hand
				if ((cx.synch_mode == SYNCH_ON_L_PRIM && sde == Krang::RIGHT) ||
					  (cx.synch_mode == SYNCH_ON_R_PRIM && sde == Krang::LEFT)){
					
					T_ref = Krang::eulerToTransform(cx.ref_poses[1-side], math::XYZ);
					T_ref = T_ref * cx.T_off_to_pri;        
				}
				wss[sde]->updateFromUIPos(T_ref, hw->fts[sde]->lastExternal,
										  nullspace_qdot_refs[sde], qdot_jacobian);
			}


			// make sure we're not going too fast
			double magn = qdot_jacobian.norm();
			if (magn > 0.5) qdot_jacobian *= (0.5 / magn);
			if (magn < .05) qdot_jacobian *= 0.0;

			// avoid joint limits
			Eigen::VectorXd qdot_avoid(7);
			Krang::computeQdotAvoidLimits(robot, *wss[sde]->arm_ids, q, qdot_avoid);

			// add qdots together to get the overall movement
			Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;

			// and apply that to the arm
			//qdot_apply << 0,0,0,0,0,0,0;
			//somatic_motor_reset(&daemon_cx, hw->arms[sde]);
			//usleep(1e6);
			if(cx.send_motor_cmds)
				somatic_motor_setvel(&daemon_cx, hw->arms[sde], qdot_apply.data(), 7);

			// Check for messages on the waypoint channels
			bool r;
			r = poll_waypnts_channel(waypts_channel[side], cx.refTraj[side]);
			if (r == true){ // update ref pose
				printw("Waypoints channel polled. \n\r");
				if (! cx.refTraj[side].empty()){
					cx.ref_poses[side] = cx.refTraj[side][0];
					cx.refTraj[side].erase(cx.refTraj[side].begin());
				}
			}

			poll_ref_vel_channel(cx.channel_ref_vel[side], cx.ref_vel[side]);

			/* Test if target position has been reached by checking if current 
			pose is within some finite neighbourhood of target pose and if 
			reached update the ref pose to next waypoint in the trajectory. */
			if (abs(cx.ref_poses[side][0] - x[0]) < 0.03 &&    // x-dir 1 cm
				abs(cx.ref_poses[side][1] - x[1]) < 0.03 &&    // y-dir 1 cm
				abs(cx.ref_poses[side][2] - x[2]) < 0.03 &&    // z-dir 1 cm 
				abs(cx.ref_poses[side][3] - x[3]) < 0.15  &&    // roll  0.3 rad
				abs(cx.ref_poses[side][4] - x[4]) < 0.15  &&    // pitch 0.3 rad
				abs(cx.ref_poses[side][5] - x[5]) < 0.15) {     // yaw   0.3 rad
			   
				if (! cx.refTraj[side].empty()){
					cx.numWayPtsReached++;
					cx.ref_poses[side] = cx.refTraj[side][0];
					cx.refTraj[side].erase(cx.refTraj[side].begin());
				}
			}
		}

		poll_cmd_channel(cx.channel_cmd);

		//std::cout<<"Left gripper pose in world frame" << std::endl<<'\r';
		//std::cout<<cx.ref_poses[L_GRIP] <<std::endl<<'\r';

		// print various things to the output
		move(0, 0);
		//clear();
		printw("----------------------------------\n");
		print_key_bindings();
		printw("----------------------------------\n");
		print_robot_state(hw, wss[Krang::LEFT], wss[Krang::RIGHT], cx);
		printw("----------------------------------\n");
		//printEvents();
		printw("----------------------------------\n");
		refresh();  // refresh the ncurses screen

		// publish the left gripper and right gripper states on the ACH channels.
		publish_to_channels(hw, wss[Krang::LEFT], wss[Krang::RIGHT], cx);

		// And sleep to fill out the loop period so we don't eat the entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));

		pthread_mutex_unlock(&mutex);

		usleep(time_sleep_usec);
	}		
}

/* ******************************************************************************************** */
/// This fxn initiailizes the deamon
void init() {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// Set pose of base in world frame as (0, 0, 0, 0, 0, 0)
	//Eigen::VectorXd basePose = Eigen::VectorXd::Zero(6);
	//robot->setConfig(Krang::dart_root_dof_ids, basePose);

	// Initialize somatic
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "workspaced";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize ACH channels for reading waypoints for left and right grippers
	enum ach_status r = ACH_OK;
	assert(ACH_OK == ach_open(&waypts_channel[L_GRIP], cx.lgrip_waypnts_channel_name, NULL ));
	assert(ACH_OK == ach_flush(&waypts_channel[L_GRIP]));

	assert(ACH_OK == ach_open(&waypts_channel[R_GRIP], cx.rgrip_waypnts_channel_name, NULL ));
	assert(ACH_OK == ach_flush(&waypts_channel[R_GRIP]));

	// Initialize ACH channels for reading reference velocity
	assert(ACH_OK == ach_open(&cx.channel_ref_vel[L_GRIP], "krang_lgrip_ref_vel", NULL ));
	assert(ACH_OK == ach_flush(&cx.channel_ref_vel[L_GRIP]));

	assert(ACH_OK == ach_open(&cx.channel_ref_vel[R_GRIP], "krang_rgrip_ref_vel", NULL ));
	assert(ACH_OK == ach_flush(&cx.channel_ref_vel[R_GRIP]));

	// Initialize ACH channels for commands
	assert(ACH_OK == ach_open(&cx.channel_cmd, g_default_cmd_chan_name, NULL ));
	assert(ACH_OK == ach_flush(&cx.channel_cmd));

	// Initialize ACH channels for publishing pose of left and right grippers
	assert(ACH_OK == ach_open(&lgrip_pose_channel, cx.lgrip_pose_channel_name, NULL));
	assert(ACH_OK == ach_flush(&lgrip_pose_channel));

	assert(ACH_OK == ach_open(&rgrip_pose_channel, cx.rgrip_pose_channel_name, NULL));
	assert(ACH_OK == ach_flush(&rgrip_pose_channel));

	// Initialize ACH channels for publishing state of left and right grippers
	assert(ACH_OK == ach_open(&lpincher_state_channel, cx.lpincher_state_channel_name, NULL));
	assert(ACH_OK == ach_flush(&lpincher_state_channel));

	assert(ACH_OK == ach_open(&rpincher_state_channel, cx.rpincher_state_channel_name, NULL));
	assert(ACH_OK == ach_flush(&rpincher_state_channel));

	// Initialize ACH channels for publishing force/torque values
	assert(ACH_OK == ach_open(&cx.channel_ft[L_GRIP], "krang_lgrip_ft", NULL));
	assert(ACH_OK == ach_flush(&cx.channel_ft[L_GRIP]));

	assert(ACH_OK == ach_open(&cx.channel_ft[R_GRIP], "krang_rgrip_ft", NULL));
	assert(ACH_OK == ach_flush(&cx.channel_ft[R_GRIP]));

	// Initialize ACH channels for publishing body state
	assert(ACH_OK == ach_open(&cx.channel_body_state, "krang_body_state", NULL));
	assert(ACH_OK == ach_flush(&cx.channel_body_state));

	// Initialize the hardware for the appropriate gripper mode
	hw = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot);
    /* Above initialization takes the first reading from IMU. Now turn off the 
     imu to avoid noise in IMU reading */
    hw->setImuOff(); cx.imu_mode = false;

	// Set up the workspace stuff
	wss[Krang::LEFT] = new Krang::WorkspaceControl(robot, Krang::LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
												   SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN,
												   COMPLIANCE_TRANSLATION_GAIN, COMPLIANCE_ORIENTATION_GAIN);
	wss[Krang::RIGHT] = new Krang::WorkspaceControl(robot, Krang::RIGHT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
													SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN,
													COMPLIANCE_TRANSLATION_GAIN, COMPLIANCE_ORIENTATION_GAIN);

	// set up the relative transform between the hands
	Trel_pri_to_off = wss[primary_hand]->Tref.inverse() * wss[off_hand]->Tref;

	// set up nullspace stuff
	//nullspace_q_refs[Krang::LEFT] = (Krang::Vector7d()   << 0, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_refs[Krang::LEFT] = (Krang::Vector7d()   << 0.88, -1.0, 0, -0.5, 0, -0.8, 0).finished();
	nullspace_q_refs[Krang::RIGHT] = (Krang::Vector7d()  << -0.88,  1.0, 0,  0.5, 0,  0.8, 0).finished();
	//nullspace_q_masks[Krang::LEFT] = (Krang::Vector7d()  << 0,    0, 0,    1, 0,    0, 0).finished();
	nullspace_q_masks[Krang::LEFT] = (Krang::Vector7d()  << 1, 0, 0, 0, 0, 0, 0).finished();
	nullspace_q_masks[Krang::RIGHT] = (Krang::Vector7d() << 1, 0, 0, 0, 0, 0, 0).finished();

	// Create a thread to wait for user input
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

    // Intial the context
    cx.send_motor_cmds = true;
    cx.synch_mode = SYNCH_OFF;
    cx.input_mode = INPUT_MODE_POSE;
    cx.compliance_mode = COMPLIANCE_ON;//COMPLIANCE_ON;
	cx.show_key_bindings = true;

	for(int i=0; i<7; i++) cx.maxCurrent[L_GRIP][i] = cx.maxCurrent[R_GRIP][i] = 0.0;

	// Send message on event channel for 'daemon initialized and running'
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
					SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	/* Store the current working directory since somatic annoyingly changes it */
	char temp[1024];
	init_wd = getcwd(temp, 1024) ? std::string( temp ) : std::string("");
	

	// parse command line arguments
	parse_args(argc, argv);

	init();		// initialize the daemon
	run();		// run the deamon
	destroy();	// destroy
}

/* ***************************************
A simple implementation of events.
Each event is just a character string.
One function records the event. Another functions prints the events. Prev events
may be over-written with new ones. Current it supports recording only last
event. It can be extended to record more events. */
typedef struct {
	char events[10][256];
	unsigned int numEvents;
	int start;
} EventQ_t;

EventQ_t g_EventQ {.events = {0}, .numEvents = 0, .start = 0};

void recordEvent(const char* event){
	strncpy(g_EventQ.events[g_EventQ.start], event, 256);
	g_EventQ.start = (g_EventQ.start+1)%10;
	g_EventQ.numEvents++;
	return;
}

void printEvents(){
	printw("EVENTS\n");
	for(int i=0; i<10; i++)
		printw("    %d: %s\n", g_EventQ.numEvents + i, g_EventQ.events[(g_EventQ.start + i)%10]);
	return;
}
