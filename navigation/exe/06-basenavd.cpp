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
 * @file basenavd.cpp
 * @author Jon Scholz, Can Erdogan
 * @date March 02, 2015
 * @brief Implements PID control to follow a trajectory. The global state of the robot is 
 * kept by combining vision data and odometry. If vision data is received, odometry is reset 
 * to stop error built-ups. 
 */

#include <iostream>
#include <fstream>
#include <deque>

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

#include <easylogging++.h>    // for logging

INITIALIZE_EASYLOGGINGPP

#include <TrajGenMultiDim.h>

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


#define SIGN(x) ((x) >=0 ? 1 : -1 )

#define IS_NEAR_ZERO(x, precision) (abs(x) < (precision))

using namespace std;

/**
 * Util to get working directory, since somatic annoyingly changes it
 */
std::string getcwd_str()
{
   char temp[1024];
   return ( getcwd(temp, 1024) ? std::string( temp ) : std::string("") );
}

/* ---------- */
/* ARGP Info  */
/* ---------- */

// context struct
typedef struct {
    // maybe set by command line arguments
    somatic_d_t d;
    somatic_d_opts_t d_opts;
    const char *opt_base_state_chan_name;
    const char *opt_base_pose_chan_name;
    const char *opt_base_waypts_chan_name;
    const char *opt_base_cmd_chan_name;
    int opt_verbosity;

    double sample_time; // time of single loop (in msecs)
    double traj_start_time; // time when target Pose is set in TrajGen
    double curr_time;    // current time in secs

    // Parameters for friction model
    double epsilon; // (rad/sec)
    double B_c_slider;    // static friction coefficient
    double B_c_tire;
    double B_v;    // viscous friction coefficient

    double integration_limit_x;
    double integration_limit_y;
    double integration_limit_theta;

    bool isStictionOn;

    double current_multiplier;
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
    case 'c':
        cx->opt_base_cmd_chan_name = strdup( arg );
        break;
    case 0:
        break;
    }
    
    somatic_d_argp_parse( key, arg, &cx->d_opts );

    return 0;
}

/* Dimenstions of the robot */
static const double wheel_base = 0.692; //< krang wheel base (width) in meters (measured)
static const double wheel_diameter = 0.536; // meters 0.543 (measured)

/* ******************************************************************************************** */
ach_channel_t state_chan, vision_chan, base_waypts_chan, cmd_chan;
cx_t cx;                            //< the arg parse struct
Krang::Hardware* krang;                //< Interface for the motor and sensors on the hardware
dynamics::SkeletonDynamics* robot;            //< the robot representation in dart

TrajGenMultiDim trajGen;        // 3-dimensional Trajectory Generator
/* ******************************************************************************************** */
bool start = false;                 //< send motor velocities
bool dbg = false;                    //< print debug output
bool update_gains = false;             //< if true, read new gains from the text file
size_t mode = 1;                    //< 0 sitting, 1 keyboard commands, 2 trajectory following
bool err_advance_waypts = true;     //< if true, allow trajectory advance on waypoint error
bool time_advance_waypts = false;    //< if true, allow trajectory advance on timeouts
bool wait_for_global_vision_msg_at_startup = false;
bool use_steering_method = false;     //< if true, use a steering controller
bool high_current_mode = false;     //< if true, allow a boost in allowed wheel currents
bool vision_updates = false;        //< if true, upate base pose from vision channel
bool odom_updates = true;            //< if true, upate base pose from wheel odometry
bool show_key_bindings = true;        //< if true, show key bindings in ncurses display

const double waypt_lin_thresh = 0.030; //0.015;         // (meters) for advancing waypoints in traj
const double waypt_rot_thresh = 0.050; //0.025;        // (radians) for advancing waypoints in traj
const double kbd_lin_incr = 1.0; // .06
const double kbd_rot_incr = 1.0;// 0.05;

bool error_integration = false;
Eigen::Vector3d int_err;            //< accumulator for integral error (x,y,theta in robot frame)
double u_hard_max = 22.0;            //< never write values higher than this
double u_spin_max = 20.0;             //< thershold on spin contribution
double u_lin_max = 16.0;             //< threshold on linear contribution
double u_high_boost = 12.0;             //< number of additional amps to add during high current mode

/* ******************************************************************************************** */
typedef Eigen::Matrix<double,6,1> Vector6d;

Vector6d refstate;                    //< reference state for controller (x,y,th,x.,y.,th.)
Vector6d state;                        //< current state (x,y,th,x.,y.,th.)
Eigen::Vector4d wheel_state;        //< wheel pos and vels in radians (lphi, lphi., rphi, rphi.)
Eigen::Vector4d last_wheel_state;     //< last wheel state used to update the state 
Eigen::VectorXd K = Eigen::VectorXd::Zero(14);    //< the gains for x and th of the state. y is ignored.
vector <Vector6d> trajectory;        //< the goal trajectory    
vector <double> timeouts;             //< associated list of timeouts (optional)
struct timespec ref_timestart;        //< timespec for trajectory timeout mode
double ref_timeout = 0.0;                 //< the timeout for the current reference 
const size_t max_traj_msg_len = 500; // 170;
size_t trajIdx = 0;

/// for debugging
deque<Vector6d> state_hist;        //< state history for filtering
const int state_hist_len = 10;    //< max number of states in history
deque<Vector6d> err_hist;        //< error history for integral control
const int err_hist_len = 10;    //< max number of states in history

FILE* log_file;                //< used to dump state and reference poses over time to plot traj. tracking
std::string init_wd;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;    //< mutex to update gains

/* Returns the angle expressed in radians between -Pi and Pi */ 
double unwrap(double angle) 
{    
    int osign = angle >= 0 ? 1 : -1;
    return osign * (fmod(abs(angle) + M_PI, 2 * M_PI) - M_PI);
}

/* Updates gains from the file. */
void readGains () {

    LOG(INFO) << "Reading Gains from file: " << "../data/gains-PID.txt";

    std::string gains_file = init_wd + "/../data/gains-PID.txt";
    ifstream file(gains_file.c_str());
    assert(file.is_open());
    char line [1024];
    // K = Eigen::VectorXd::Zero(7);
    file.getline(line, 1024);
    while (line[0] == '#')
        file.getline(line, 1024);
    std::stringstream stream(line, std::stringstream::in);
    size_t i = 0;
    double newDouble;
    while ((i < 14) && (stream >> newDouble)) K(i++) = newDouble;
    file.close();

    cx.B_c_slider = K[8];
    cx.B_c_tire = K[9];
    cx.epsilon = K[10];
    cx.integration_limit_x = K[11];
    cx.integration_limit_y = K[11];
    cx.integration_limit_theta = K[12];

    cx.current_multiplier = K[13];
}

/* Returns frictional control force to tackle friction. 
 *  thetaDot: [IN] Angular velocity
 *  B_c: [IN] Viscous friction co-efficient*/
double getFrictionalTorque(double thetaDot, double B_c) {
    if (abs(thetaDot) <= cx.epsilon)
        return (B_c * thetaDot)/cx.epsilon;
    else
        return B_c * SIGN(thetaDot) + cx.B_v * thetaDot;
}

/* A single setter for refstate, so we have the chance to use as
 * trigger for other events (flags and stuff) 
 *  newRef: [IN] The new reference state 
 *  from_current_state: [IN] If true, trajectory is planned from current 
*       state of the robot. Otherwise, from current reference point. */
void setReference(Vector6d& newRef, bool from_current_state=false) {
    refstate = newRef;
    int_err.setZero();

    // To avoid discontinuity in control forces applied plan trajectory from
    // current reference point
    Vector6d refStateTraj = 
                trajGen.getReferenceState(cx.curr_time - cx.traj_start_time);

    if(from_current_state)
        trajGen.setTargetState(state, newRef);
    else
        trajGen.setTargetState(refStateTraj, newRef);
        
    cx.traj_start_time = cx.curr_time;
}

/* Updates the pose params of the robots state from vision messages
 * 
 * @param block: if true, wait for a message to come for 5 seconds in before returning
 * @param reset_reference: if true, set the reference to the new state if 
 *         a message is received.  This is a safety for keeping the robot from
 *         lurching when a big state update comes in.  Should only be false
 *         if polling to fix small errors while executing a trajectory.  */
void poll_vision_channel(bool block, bool reset_reference=true)
{
    // Receive the current state from vision
    // double rtraj[1][3] = {0, 0, 0};
    int bufsize = 2 * sizeof(int) + 3 * sizeof(double); // krang pose message size
    char buf[bufsize];

    size_t frame_size = 0;

    /* Get the message */
    int options = block ? ACH_O_WAIT | ACH_O_LAST : ACH_O_LAST;
    enum ach_status r = ach_get(&vision_chan, &buf, bufsize, &frame_size, NULL, ACH_O_LAST);

    if(!(r == ACH_OK || r == ACH_MISSED_FRAME)) 
        return;

    // We don't have to update if the message we got was last one. That basically 
    // means message was published long ago and robot is no more under camera views.
    // So, take value after 100 msec. Vision channel definitely publishes at rate higher
    // that.
    usleep(100000);
    r = ach_get(&vision_chan, &buf, bufsize, &frame_size, NULL, ACH_O_LAST);

    if(!(r == ACH_OK || r == ACH_MISSED_FRAME)) 
        return;

    // Set the current state to vision data
    Eigen::MatrixXd M = deserialize_to_Matrix(buf);
    state.head(3) = M.row(0).head(3); // M.topLeftCorner(1,3);
    state(3) = state(4) = state(5) = 0.0; // set vels to zero (will be over-written by odometry)
    // cout << "Updated state from vision message: " << state.transpose() << endl;
    // printw("Updated state from vision message: \n"); PRINT_VECTOR(state.transpose())

    // reset odometry
    last_wheel_state = wheel_state;

    // never change the state without resetting the reference!(?)
    if (reset_reference)
        setReference(state);
}

/* Coverts command to corresponding command string.
 *  cmdCode: [IN] The command code*/
const char* cmd_code_to_str(int cmdCode) {
    switch(cmdCode) {
        case 0: return "Set odometry updating";   break;
        case 1: return "Set vision upating";      break;
        case 2: return "Update from vision once"; break;
        case 3: return "Set error advance mode";  break;
        case 4: return "Set time advance mode";   break;
        case 5: return "Halt";                    break;
        case 6: return "Set advance mode";                    break;
        default: return "Unknown command code";   break;
    }
}

/* Reads the command channel and takes action. When, this list is modified 
 * 'Krang Software Manual' document should be updated.
 * 
 * Command messages are 1x2 trajectory messages that get rounded
 * to ints, and are parsed as (cmd-code, cmd-value) tuples. */
void poll_cmd_channel(){

    size_t frame_size = 0;  
    char buf[2 * sizeof(int) + 2 * sizeof(double)];

    enum ach_status r = ach_get(&cmd_chan, buf, sizeof(buf), &frame_size, NULL, ACH_O_LAST);
    if(r != ACH_OK && r != ACH_MISSED_FRAME) return;

    if(r == ACH_MISSED_FRAME) LOG(DEBUG) << "Frame has been missed.";

    Eigen::MatrixXd cmdM = deserialize_to_Matrix(buf);
    if(cmdM.size() != 2)
        printw("Error at Line %d: Invalid message on command code\n\r", __LINE__);

    int cmdCode = int(cmdM(0) + 0.5);   // round the value
    int val = int(cmdM(1) + 0.5);       // round the value

    LOG(INFO) << "Command Received. Cmd Code: " << cmdCode << " (" 
        << cmd_code_to_str(cmdCode) << ") Value: " << val;

    switch(cmdCode){ 
        case 0: // set odometry updating
            if (val == 0 || val == 1)
                odom_updates = val;
            else
                printw("Error at Line %d: Invalid message on command code\n\r", __LINE__);
            break;
        case 1: // toggle vision updating
            if (val == 0 || val == 1)
                vision_updates = val;
            else
                printw("Error at Line %d: Invalid message on command code\n\r", __LINE__);
            break;
        case 2: // single vision update
            poll_vision_channel(true, false);
            break;
        case 3: // toggle error advance mode
            if (val == 0 || val == 1)
                err_advance_waypts = val;
            else
                printw("Error at Line %d: Invalid message on command code\n\r", __LINE__);                
            break;
        case 4: // toggle time advance mode
            if (val == 0 || val == 1)
                time_advance_waypts = val;
            else
                printw("Error at Line %d: Invalid message on command code\n\r", __LINE__);                
            break;
        case 5: // halt
            setReference(state, true);
            trajectory.clear();
            break;
        case 6: // set advance mode
            if (val >=0 && val <=3) {
                err_advance_waypts = val & 0b10; // 2nd bit
                time_advance_waypts = val & 0b01; // 1st bit
            }
            else
                printw("Error at Line %d: Invalid message on command code\n\r", __LINE__);                
            break;
        default: 
            break;
    }
    return;
}

/* Gets the joint values from the encoders and the imu and compute the center of mass as well 
 *
 * wheel_state is a 4-vector containing:
 *  linear position (total distance traveled since start; average of both wheels)
 *  angular position (difference of wheel dist over wheelbase)
 *  linear velocity (current linear velocity; average of both wheels)
 *  angular velocity (difference of wheel linear velocities over wheelbase)
 *
 *  dt: [IN] The time elapsed since this function was last called.
 *  wheel_state: [OUT] The state of the wheels */
void updateWheelsState(double dt, Eigen::Vector4d& wheel_state) {
    // Model constants
    static const double wheel_radius = wheel_diameter/2;

    // Update the sensor information
    //krang->updateSensors(dt);
    somatic_motor_update(krang->daemon_cx, krang->amc);
    
    // Change the wheel values from radians to meters
    double pleft  = krang->amc->pos[0] * wheel_radius; //< traversed distances for left wheel in meters
    double pright = krang->amc->pos[1] * wheel_radius; //< traversed distances for right wheel in meters
    double vleft  = krang->amc->vel[0] * wheel_radius; //< left wheel velocity in m/s <-- cafeful: noisy!
    double vright = krang->amc->vel[1] * wheel_radius; //< right wheel velocity in m/s <-- cafeful: noisy!

    // Set the state
    wheel_state[0] = (pleft + pright)/2.0; // + krang->imu;
    wheel_state[1] = (pright - pleft)/wheel_base;

    // "correct" version:
    wheel_state[2] = (vleft + vright)/2.0;
    wheel_state[3] = (vright - vleft)/wheel_base;

    // finite diff version:
    // wheel_state[2] = (wheel_state[0] - last_wheel_state[0]) / dt; // finite diff. rather than amc vels b/c of noise
    // wheel_state[3] = (wheel_state[1] - last_wheel_state[1]) / dt; // finite diff. rather than amc vels b/c of noise

    // wheel_state[2] = 0.;
    // wheel_state[3] = 0.;
}

/* ******************************************************************************************** */

/* Update the state using the wheels information with Euler approximation (we use the last
 * theta with the forward motion to reflect changes on x and y axes)
 *  wheel_state: [IN]
 *  last_wheel_state: [IN]
 *  state: [OUT]
 *
 * State is a 6-vector of the form [x, y, theta, xdot, ydot, thetadot] in world frame. */
void updateState(Eigen::Vector4d& wheel_state, Eigen::Vector4d& last_wheel_state, Vector6d& state) {

    // Compute the change in forward direction and orientation
    double dlin   = wheel_state[0] - last_wheel_state[0]; // linear distance covered (in *some* direction)
    double dtheta = wheel_state[1] - last_wheel_state[1]; // angular distance covered
    double last_theta = state[2]; // last robot heading is whatever theta was before we update it

    state[0] += dlin * cos(last_theta);                //< x
    state[1] += dlin * sin(last_theta);                //< y
    state[2] += dtheta;                                //< theta
    state[2] = unwrap(state[2]);
    state[3] = wheel_state[2] * cos(last_theta);     //< xdot
    state[4] = wheel_state[2] * sin(last_theta);     //< ydot
    state[5] = wheel_state[3];                        //< thetadot

    /// for debugging
    state_hist.push_front(state);
    if (state_hist.size() > state_hist_len)
        state_hist.pop_back();
}

/* Computes the error between the provided state and reference state vectors.
 * Both are assumed to be in the world (or odom) frame.
 *  state: [IN] The current state of the robot base in world frame.
 *  refstate: [IN] The reference state of the robot base in world frame.
 *  rotate: [IN] If True, errors are in robot frame. Otherwise, in world frame.
 *
 * Returns error vector of length 6 of form 
 *   [delta_x, delta_y, delta_theta, delta_xDot, delta_yDot, delta_thetaDot] */
Vector6d computeError(const Vector6d& state, const Vector6d& refstate, 
    bool rotate=true) {
    // Error vector in world (or odom) frame, which is where state and refstate are defined
    Vector6d err = refstate - state;

    err[2] = unwrap(err[2]);             //< unwrap angular portion of error
    Eigen::Rotation2Dd rot(state[2]);     //< used for rotating errors between robot and world frame

    // strict steering method:
    if (use_steering_method) {
        double refpos_angle = atan2(err[1], err[0]);                     //< angle towards current waypoint
        double refpos_dist = err.segment(0,2).norm();                     //< linear distance to current waypoint
        double refpos_angle_err = unwrap(refpos_angle - state[2]);         //< heading error 
        double ref_xerr_robot = (rot.inverse() * err.segment(0,2))[0];     //< x error in robot frame

        if (refpos_dist > waypt_lin_thresh * 3    &&        //< if we're far from the waypoint
           (ref_xerr_robot > 0 || ref_xerr_robot < -waypt_lin_thresh * 3)) //< but not just a little ahead of it (ignores minor overshooting)
            err[2] = refpos_angle_err;                         //< then only worry about turning towards it
        //printw("refpos_dist: %lf (thresh: %lf)\n", refpos_dist, waypt_lin_thresh*3);
        //printw("ref_xerr_robot: %lf(thresh: %lf)\n", ref_xerr_robot, -waypt_lin_thresh*3);
        //printw("refpos_angle_err: %lf\n", refpos_angle_err);
        //printw("steering cond test: %d\n", refpos_dist > waypt_lin_thresh * 3 && ref_xerr_robot > -waypt_lin_thresh * 3);
        if (err[2] > waypt_rot_thresh * 3)
            err.segment(0,2).setZero();                     //< zero all translation controls if we're rotating
    }

    // if requested, convert error vector to robot frame, which is where we do control
    if (rotate) {
        err.segment(0,2) = rot.inverse() * err.segment(0,2); //< rotate position error to robot frame
        err.segment(3,2) = rot.inverse() * err.segment(3,2); //< rotate velocity error to robot frame    
    }

    // fprintf(log_file, "%2.4lf, %2.4lf, %2.4lf, %2.4lf, %2.4lf, %2.4lf, ", 
    //     state(0), state(1), state(2), state(3), state(4), state(5));
    // fprintf(log_file, "%2.4lf, %2.4lf, %2.4lf, %2.4lf, %2.4lf, %2.4lf,  \n", 
    //     err(0), err(1), err(2), err(3), err(4), err(5));
    // fflush(log_file);
    return err;
}

/* Computes Torques for both wheels.
 *  state: [IN] The current state of the robot base in world frame
 *  dt: [IN] time elapsed since application of previous control input
 *  ul: [OUT] Torque to be applied for left wheel
 *  ur: [OUT] Torque to be applied for left wheel
 */
void computeTorques(const Vector6d& state, double dt, double& ul, double& ur) {
    
    Vector6d refStateTraj = 
                trajGen.getReferenceState(cx.curr_time - cx.traj_start_time);

    //Vector6d err_robot = computeError(state, refstate, true);
    Vector6d err_robot = computeError(state, refStateTraj, true);

    // Set reference based on the mode
    if(mode == 1 || mode == 0) ul = ur = 0.0;

    if (error_integration) {
        Vector6d cur_err = err_robot * dt;
        int_err += cur_err.head(3);

        if (abs(int_err[0]) >= cx.integration_limit_x)
            int_err[0] = SIGN(int_err[0]) * cx.integration_limit_x;

        if (abs(int_err[1]) >= cx.integration_limit_y)
            int_err[1] = SIGN(int_err[1]) * cx.integration_limit_y;
        
        if (abs(int_err[2]) >= cx.integration_limit_theta)
            int_err[2] = SIGN(int_err[2]) * cx.integration_limit_theta;
    }

    if (dbg) {
        printw("Reference Error (robot frame): "); PRINT_VECTOR(err_robot);
        printw("Integrated errors: "); PRINT_VECTOR(int_err);
        printw("state: "); PRINT_VECTOR(state);
    }

    /* Get the velocity in robot frame */
    double xDot_robot = state[3] * cos(state[2]) + state[4] * sin(state[2]);

    // Compute the forward and rotation torques
    // Note: K is organized as [linearP linearI linearD angularP angularI angularD k_fri]
    //          err_robot is organized as [x, y, theta, xdot, ydot, thetadot]
    //double u_x = K[0] * err_robot[0] + K[1] * int_err[0] + K[2] * err_robot[3];
    double u_x = K[0] * err_robot[0] + K[1] * int_err[0] + K[2] * err_robot[3];
    // u_x += k_fri * (err_robot[0] > 0 ? 1 : -1); // static friction term
    //u_x += k_fri * err_robot[0]; // static friction term

    //double u_theta = K[3] * err_robot[2] + K[4] * int_err[2] + K[5] * err_robot[5];
    double u_theta = K[3] * err_robot[2] + K[4] * int_err[2] + K[5] * (-state[5]);
    //u_theta += k_fri * err_robot[2]; // static friction term

    //if(dbg) {
    //    LOG(DEBUG) << "u_x: " << u_x << " u_theta: " << u_theta;
    //    LOG(DEBUG) << "Velocity: Lin = " << -err_robot[3] 
    //               << " Ang = " << -err_robot[5];
    //    LOG(DEBUG) << "Diffential component: Lin = " << K[2] * err_robot[3]
    //               << " Ang = " << K[5] * (-state[5]);
    //} 

    // Limit the output torques
    double u_boost = high_current_mode ? u_high_boost : 0.0;
    
    u_theta = max(-u_spin_max-u_boost, min(u_spin_max + u_boost, u_theta));
    u_x     = max(-u_lin_max-u_boost, min(u_lin_max + u_boost, u_x));
    
    /* Static friction due to slide */
    u_x = u_x + getFrictionalTorque(u_x, cx.B_c_slider);
    u_theta = u_theta + getFrictionalTorque(u_theta, cx.B_c_slider);

    if(dbg) printw("u_x: %lf, u_theta: %lf\n", u_x, u_theta);

    double ul1 = u_x - u_theta;
    double ur1 = u_x + u_theta;

    // Get the frictional component
    // Convert to robot frame (des mean desired)
    double v_des = refStateTraj[3] * cos(state[2]) + refStateTraj[4] * sin(state[2]);
    double thetaDot_des = refStateTraj[5];
    double v_r_des = v_des + thetaDot_des * wheel_base/2;
    double v_l_des = v_des - thetaDot_des * wheel_base/2;

    double thetaDot_r_des = v_r_des * 2/wheel_diameter;
    double thetaDot_l_des = v_l_des * 2/wheel_diameter;

    if(dbg) printw("ul: %lf, ur (before friction): %lf\n", ul1, ur1);

    /* Static friction due to wheels */
    ul = ul1 + getFrictionalTorque(ul1, cx.B_c_tire); // thetaDot_l_des);
    ur = ur1 + getFrictionalTorque(ur1, cx.B_c_tire); // thetaDot_r_des);

    if(dbg) printw("ul: %lf, ur (after friction): %lf\n", ul, ur);

    /* Add stiction only if control force is good enough. Otherwise, it leads to 
    vibration as stiction changes signs around reference point */
    if(cx.isStictionOn) {
        // Stiction-friction compensation: adaptive p-gain: falls inversely with vel to get robot started
        double vel_lin = wheel_state[2];
        double vel_ang = wheel_state[3];
        double vright = vel_lin + vel_ang * wheel_base/2;
        double vleft = vel_lin - vel_ang * wheel_base/2;

        //double velsq = state.segment(3,3).squaredNorm(); // total velocity magnitude
        // function favours lesser vel and more control to be applied.
        double temp1 = 1/(1 + exp(-abs(ur))) - 0.5;
        double stiction_r = SIGN(ur) * abs(temp1) * K[6] * exp(-K[7] * abs(vright)); //< exponentially falls off as robot gains speed
        temp1 = 1/(1 + exp(-abs(ul))) - 0.5;
        double stiction_l = SIGN(ul) * abs(temp1) * K[6] * exp(-K[7] * abs(vleft));

        ur = ur + stiction_r;
        ul = ul + stiction_l;
    }

    /* Limit the values */
    ul = max(-u_hard_max-u_boost, min(u_hard_max+u_boost, ul));
    ur = max(-u_hard_max-u_boost, min(u_hard_max+u_boost, ur));

    /*
    if (abs(ul) > 1)
        ul = ul + SIGN(ul) * cx.B_c;

    if (abs(ur) > 1)
        ur = ur + SIGN(ur) * cx.B_c;
    */

    ul = cx.current_multiplier * ul;
    ur = cx.current_multiplier * ur;

}

/* Updates the trajectory using the waypoints channel */
void updateTrajectory () {
    // allocate a buffer big enough for the incoming trjajectory
    // size: max_len x [two ints for shape, 3 doubles for pose, 1 double for timeout]
    char buf[2*sizeof(int) + max_traj_msg_len * (3*sizeof(double) + 1*sizeof(double))];

    // Check if a message is received
    size_t frame_size = 0;
    struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
    ach_status_t r = ach_get(&base_waypts_chan, &buf, sizeof(buf), 
        &frame_size, &abstimeout, ACH_O_LAST);
    if(!(r == ACH_OK || r == ACH_MISSED_FRAME)) return;

    int n, m;
    memcpy(&n, &buf[0], sizeof(int));
    memcpy(&m, &buf[sizeof(int)], sizeof(int));

    Eigen::MatrixXdr M;
    M.resize(n, m);
    memcpy(M.data(), &buf[2*sizeof(int)], sizeof(double) * n * m);

    Eigen::MatrixXd poses = M;

    // return if received trajectory is empty
    if (poses.rows() == 0) return;

    // Fill the trajectory data (3 doubles, 8 bytes, for each traj. point)
    // Update: now that we're unpacking to Eigen, could plug matrix in for 
    // traj and create refState (with vels) in run loop, but why bother?
    trajectory.clear();
    timeouts.clear();
    ref_timestart = aa_tm_now();

    for(size_t i = 0; i < poses.rows(); i++) {
        Vector6d newRefState;
        newRefState << poses(i,0), poses(i,1), poses(i,2), 0.0, 0.0, 0.0;
        trajectory.push_back(newRefState);

        // if a 4th element was provided, it's a timestamp
        if (poses.cols() > 3)
            timeouts.push_back(poses(i,3));
        else
            timeouts.push_back(0.0);
    }
    static int tctr = 0;

    if (dbg) {
        printw("updating trajectory from poses:\n"); PRINT_MAT(poses)
        printw("associated timeouts:\n"); 
        for (int i=0; i<timeouts.size(); i++)
            printw(" %lf ", timeouts[i]);
    }
    tctr++;

    // Update the reference state
    trajIdx = 0;
    setReference(trajectory[0]);
}

/* Prints the keyboard keys and \corresponding actions to the console */
void print_key_bindings(){
    const char *allstr =
     "'b'  : hide key bindings                                           \n\r"
     "' '  : toggle ON and OFF sending commands to motors                \n\r"
     "'q'  : Quit                                                        \n\r"
     "'r'  : Reset Reference                                             \n\r"
     "' '  : Start/Stop motor commands                                   \n\r"
     "'d'  : Toggle debug output                                         \n\r"
     "'e'  : Toggle error waypoint advance mode                          \n\r"
     "'t'  : Toggle timeout waypoint advance mode                        \n\r"
     "'u'  : Toggle error integration                                    \n\r"
     "'v'  : Toggle vision updates (polling base pose)                   \n\r"
     "'o'  : Toggle odometry                                             \n\r"
     "'g'  : Re-read gains from file (gains-PID.txt)                     \n\r"
     "'s'  : Use steering-method in controller (turns towards reference) \n\r"
     "'1'  : Set Mode: OFF                                               \n\r"
     "'2'  : Set Mode: KEYBOARD                                          \n\r"
     "'3'  : Set Mode: TRAJECTORY                                        \n\r"
     "'j'  : Keyboard control: turn left                                 \n\r"
     "'l'  : Keyboard control: turn right                                \n\r"
     "'i'  : Keyboard control: drive forward                             \n\r"
     "'k'  : Keyboard control: drive backward                            \n\r"
     "'h'  : High-current mode (increases current limits; be careful!)   \n\r"
     "'f'  : Toggle Compensate for Sticition friction                    \n\r";
     
    printw("\t\t[KEY BINDINGS]\n\r%s\r", 
        show_key_bindings ? allstr : "'b': show key bindings\n\r");
    return;
}

/* Reads a character input from stdin and takes appropriate action. See 
 * function print_key_bindings() for the key-bindings. 
 * It uses getch() to read the character. Call the function timeout to make it 
 * non-blocking. */
void kbhit() {
    Vector6d newRefState;
    char ch = getch();
    switch (ch) {
        case 'q': 
            somatic_sig_received = true; break;
        case '1': case '2': case '3':
            mode = ch - 48;
            break;
        case 'd':
            dbg = !dbg;
            break;
        case 'e':
            err_advance_waypts = !err_advance_waypts;
            break;
        case 't':
            time_advance_waypts = !time_advance_waypts;
            break;
        case 'v':
            //vision_updates = !vision_updates;
            poll_vision_channel(true, false);
            break;
        case 'o':
            odom_updates = !odom_updates;
            break;
        case ' ':
            start = !start;
            break;
        case 'f':
            cx.isStictionOn = !cx.isStictionOn;     //< if true, read new gains from the text file
            break;
        case 'g':
            update_gains = true;     //< if true, read new gains from the text file
            break;
        case 'r':
            setReference(state, true);
            trajectory.clear();
            break;
        case 'i':
            if (mode != MODE_KBD) break;
            // input is in the the robot frame, so we rotate to world for refstate
            newRefState = refstate;
            newRefState(0) += kbd_lin_incr * cos(state[2]);
            newRefState(1) += kbd_lin_incr * sin(state[2]);
            setReference(newRefState);
            int_err.setZero();
            break;
        case 'k':
            if (mode != MODE_KBD) break;
            // input is in the the robot frame, so we rotate to world for refstate
            newRefState = refstate;
            newRefState(0) -= kbd_lin_incr * cos(state[2]);
            newRefState(1) -= kbd_lin_incr * sin(state[2]);
            setReference(newRefState);
            int_err.setZero();
            break;
        case 'j':
            if (mode != MODE_KBD) break;
            newRefState = refstate;
            newRefState(2) = unwrap(newRefState(2) + kbd_rot_incr);
            setReference(newRefState);
            int_err.setZero();
            break;
        case 'l':
            if (mode != MODE_KBD) break;
            newRefState = refstate;
            newRefState(2) = unwrap(newRefState(2) - kbd_rot_incr);
            setReference(newRefState);
            int_err.setZero();
            break;
        case 'u':
            error_integration = !error_integration;
            break;
        case 'b':
            show_key_bindings = !show_key_bindings;
            if (!show_key_bindings)
                clear(); // clears extra space from display
            break;
        case 's':
            use_steering_method = !use_steering_method;
            break;
        case 'h':
            high_current_mode = !high_current_mode;
            break;
        default:
            break;
    }

}

/* Prints the current status of the machine */
void print_status(const Krang::Hardware* hw, const cx_t& cx) {
    printw("\t\t[STATUS]\n");
    
    printw("Sending motor commands:\t\t");
    printw(start ? "ON\n" : "OFF\n");
    
    printw("Control mode:\t\t\t");
    switch (mode) {
        case 1: printw("OFF\n"); break;
        case 2: printw("KEYBOARD\n"); break;
        case 3: printw("TRAJECTORY-FOLLOWING\n"); break;
    }

    printw("Steering method:\t\t");
    printw(use_steering_method ? "ON\n" : "OFF\n");

    printw("Waypoint error advance:\t\t");
    printw(err_advance_waypts ? "ON\n" : "OFF\n");

    printw("Waypoint time advance:\t\t");
    printw(time_advance_waypts ? "ON\n" : "OFF\n");

    printw("Error integration:\t\t");
    printw(error_integration ? "ON\n" : "OFF\n");

    printw("Vision updates:\t\t\t");
    printw(vision_updates ? "ON\n" : "OFF\n");

    printw("Odometry updates:\t\t");
    printw(odom_updates ? "ON\n" : "OFF\n");

    printw("Debug output:\t\t\t");
    printw(dbg ? "ON\n" : "OFF\n");

    printw("Stiction Friction:\t\t");
    printw(cx.isStictionOn ? "ON\n" : "OFF\n");

    // Integration is not really appropriate in this controller
    // printw("do integration: %d\n", error_integration);

    printw("----------------------------------\n");
    printw("Gains (linear P I D, angular P I D, P-booster):\n\t");  
    PRINT_VECTOR(K)

    printw("Base state ([x,y,t,x.,y.,t.] in world frame):\n\t");
    PRINT_VECTOR(state)

    printw("Reference state ([x,y,t,x.,y.,t.] in world frame):\n\t");
    PRINT_VECTOR(refstate)
    printw("timeout: (%2.2lf)\n", ref_timeout);

    printw("TrajGen Reference state\n");
    Vector6d trajGenRefState = trajGen.getReferenceState(cx.curr_time - cx.traj_start_time);
    PRINT_VECTOR(trajGenRefState)

    printw("Wheel State (:\n\t");
    PRINT_VECTOR(wheel_state)

    printw("Current limits: %2.2lf %2.2lf %2.2lf (lin,ang,hard)\n", 
        high_current_mode ? u_lin_max + u_high_boost : u_lin_max, 
        high_current_mode ? u_spin_max + u_high_boost : u_spin_max, 
        high_current_mode ? u_hard_max + u_high_boost : u_hard_max);

    printw("Trajectory index: [%d/%d]\n", trajIdx, trajectory.size()-1);

    printw("Forward Speed %f m/sec\n", wheel_state[2]);
    printw("Angular Speed %f rad/sec\n", wheel_state[3]);

    printw("Sample Loop Time: %.2f msecs\n", cx.sample_time * 1000);
}

void run() {
    // Send a message; set the event code and the priority
    somatic_d_event(&cx.d, SOMATIC__EVENT__PRIORITIES__NOTICE, 
            SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    // Continue processing data until stop received
    struct timespec t_now, t_prev = aa_tm_now();
    struct timespec t_prev_print = aa_tm_now();
    struct timespec t_prev_kbhit = aa_tm_now();

    while(!somatic_sig_received) {

        // Get the current time and compute the time difference and update the prev. time
        t_now = aa_tm_now();                        
        double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));    
        t_prev = t_now;
        
        // take input from keyboard and cmd channel every 100 msec
        if((double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev_kbhit)) > 0.1) {
            kbhit();
            poll_cmd_channel(); // poll cmd_chan for updates 

            t_prev_kbhit = t_now;
        }

        // Read the gains if requested by user
        if(update_gains) {
            readGains();
            update_gains = false;
        }

        cx.sample_time = dt;
        cx.curr_time = (double)aa_tm_timespec2sec(t_now);

        // poll krang_vision for updates to state before odometry updates
        if (vision_updates) 
            poll_vision_channel(false, false);

        // Get the state and update odometry
        if (odom_updates) {
            last_wheel_state = wheel_state;
            updateWheelsState(dt, wheel_state); 
            updateState(wheel_state, last_wheel_state, state);
        }

        // Send the state
        char buf[56]; // two ints and 6 doubles
        serialize_from_Matrix(state.transpose(), buf); // send state as row-vector like vision
        ach_put(&state_chan, &buf, 56);

        // Compute and apply the torques based on the state and the mode
        double u[2] = {0.0, 0.0}; // left and right control inputs
        if(start)
            computeTorques(state, dt, u[0], u[1]);
        somatic_motor_cmd(&cx.d, krang->amc, 
            SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, u, 2, NULL);
                
        Vector6d err = computeError(state, refstate, true);
        double lin_error = abs(err[0]);     //< linear distance in controllable direction
        double rot_error = abs(err[2]);     //< angular distance to current waypoint

        bool reached = (lin_error < waypt_lin_thresh) &&
                                //IS_NEAR_ZERO(wheel_state[2], 0.001) &&    // linear velocity
                                (rot_error < waypt_rot_thresh)
                                //&& IS_NEAR_ZERO(wheel_state[3], 0.001    // angular velocity
                                ;

        // Update the reference state if necessary
        if(mode == MODE_TRAJ) {
            updateTrajectory(); // Check if a new trajectory data is given

            double ref_dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, ref_timestart));
            bool timeout = ref_dt > ref_timeout;
            
            if ((reached && err_advance_waypts) || (timeout && time_advance_waypts)) {
                // reset integral stuff whenever we reach any reference
                //int_err.setZero();

                // advance if trajectory is not empty and we haven't reached the last waypoint
                if(!trajectory.empty() && trajIdx < trajectory.size() - 1) {
                    setReference(trajectory[++trajIdx]);

                    ref_timeout = timeouts[trajIdx];
                    ref_timestart = t_now;
                }
            }
        }

        // print every 200 msec
        if((double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev_print)) > 0.2) {
            // clear();
            move(0, 0);
            printw("----------  Basenavd  ------------\n");
            print_key_bindings();
            printw("----------------------------------\n");
            print_status(krang, cx);
            printw("----------------------------------\n");
            printw("\t\t[DEBUG MESSAGES]\n");
            t_prev_print = t_now;

            refresh();
        }

        if(dbg) {
            printw("sending wheel currents: [%f, %f]\n", u[0], u[1]);
            printw("lin_error: %f (thresh=%f)\n", lin_error, waypt_lin_thresh); 
            printw("rot_error: %f (thresh=%f)\n", rot_error, waypt_rot_thresh);

            printw("reached = %s\n", reached ? "TRUE" : "FALSE");
            printw("reached: %d, xreached: %d, threached: %d\n", reached,
                (lin_error < waypt_lin_thresh), (rot_error < waypt_rot_thresh));
            LOG(DEBUG) << "sampling interval: " << cx.sample_time << " secs";
        }
    } // while loop

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
    cx.opt_base_cmd_chan_name = "cmd_basenavd";
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
        fprintf(stderr, "basenavd command channel:      %s\n", cx.opt_base_cmd_chan_name);
        fprintf(stderr,"-------\n");
   }
}

/* ******************************************************************************************** */
void init()
{
    // Open the log file (this is just for debugging)
    log_file = fopen((init_wd + "/basenavd_log.txt").c_str(), "w+");
    assert((log_file != NULL) && "Could not open the file");

    cx.sample_time = 0;
    cx.traj_start_time = 0;
    cx.curr_time = 0;

    cx.epsilon = 0.1;
    cx.B_c_tire = 0;
    cx.B_c_slider = 0;
    cx.B_v = 0;
    cx.integration_limit_x = 0.1;
    cx.integration_limit_y = 0.1;
    cx.integration_limit_theta = 0.6;

    cx.isStictionOn = true;

    cx.current_multiplier = 1.0;

    // Read the gains
    readGains();

    // initialize errors (not really used anymore)
    int_err.setZero();
    error_integration = true;

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

    // Open channel to receive basenavd commands
    r = ach_open(&cmd_chan, cx.opt_base_cmd_chan_name, NULL );
    assert(ACH_OK == r);
    r = ach_flush(&cmd_chan);

    // Receive the current state from vision
    if (wait_for_global_vision_msg_at_startup) {
        cout << "waiting for initial vision data on vision channel: " << endl;
        poll_vision_channel(true, true);
    }

    // Set the state, refstate and limits
    updateWheelsState(1e-3, wheel_state);
    last_wheel_state = wheel_state;
    updateState(wheel_state, last_wheel_state, state);
    setReference(state);

    // general curses stuff
    initscr();
    clear();+
    noecho();                   // do not echo input to the screen
    cbreak();                   // do not buffer by line (receive characters immediately)
    timeout(0);                 // non-blocking getch

    // Create a thread to wait for user input to begin balancing
    //pthread_t kbhitThread;
    //pthread_create(&kbhitThread, NULL, &kbhit, NULL);
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

    /* Setup the logger */
    el::Configurations defaultConf;
    defaultConf.setToDefault();
    defaultConf.set(el::Level::Debug,
        el::ConfigurationType::ToStandardOutput, "false");
    defaultConf.setGlobally(
        el::ConfigurationType::Format, "%datetime %level %line: %msg");
    el::Loggers::reconfigureLogger("default", defaultConf);

    LOG(INFO) << "basenavd has started.";

    init_wd = getcwd_str();

    // parse command line arguments
    parse_args(argc, argv);

    // Load the world and the robot
    DartLoader dl;
    simulation::World* world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
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