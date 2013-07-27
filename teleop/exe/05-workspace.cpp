/**
 * @file 05-workspace.cpp
 * @author Saul Reynolds-Haertle, Can Erdogan
 * @date July 25, 2013
 * @brief This file demonstrates workspace control running on
 * Krang. It will probably be a workhorse for general tasks, including
 * demonstrations.
 */

#include "kore.h"

#include <iostream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <math/UtilsRotation.h>
#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>

#include "util.h"

using namespace Krang;

bool myDebug;
std::vector <double> data;

/* ********************************************************************************************* */
/// Home configuration for the arms
const Eigen::VectorXd HOMECONFIGL = (VectorXd(7) << 1.102, -0.589, 0.000, -1.339, 0.000, -0.959, -1.000).finished();

// initializers for the workspace control constants
const double K_WORKERR_P_INIT = 1.00;
const double NULLSPACE_GAIN_INIT = 0.0;
const Eigen::VectorXd NULLSPACE_Q_REF_INIT = 
		(VectorXd(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
const double DAMPING_GAIN_INIT = 0.005;

// various rate limits - be nice to other programs
const double LOOP_FREQUENCY = 300.0;

// constants for helping with joint limits
const Eigen::VectorXd JOINTLIMIT_REGIONSIZE = (VectorXd(7) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished();
const double JOINTLIMIT_MAXVEL = 1.0; // move joints up to this fast to avoid limits
const double JOINTLIMIT_GAIN = 0.01; // how "wide" the curve is that we use to avoid limits

/* ********************************************************************************************* */
class WorkspaceControl {
public:
	// constructor and destructor
	WorkspaceControl(somatic_d_t* _daemon_cx,
									 const char* _spacenav_chan_name,
									 std::vector<int>* _arm_ids,
									 kinematics::BodyNode* _endeff,
									 double _K_position_p = 1.0);
	~WorkspaceControl();

	// member functions
	bool getSpaceNav(Eigen::VectorXd& config);
	void updateReferenceFromXdotInput(const VectorXd& xdot, const double dt);
	void getXdotFromXref(VectorXd& xdot);
	void getQdotFromXdot(const VectorXd& xdot, const VectorXd& qdot_nullspace, VectorXd& qdot);

	// member variables
	Eigen::Matrix4d t_ref;
	ach_channel_t spacenav_chan;
	somatic_d_t* daemon_cx;
	kinematics::BodyNode* endeff;
	std::vector<int>* arm_ids;

	// configuration stuff
	double K_workerr_p;        // p constant for the PID controller that produces workspace movements
	double nullspace_gain;      // p constant for the PID controller that pulls the arm through nullspace to the secondary goal
	VectorXd nullspace_q_ref;  // secondary goal configuration which we use the nullspace to move toward
	double damping_gain;       // how powerful the damping is on our damped jacobian
};

/* ********************************************************************************************* */
// state variables for the daemon - one daemon context and one
// hardware object
somatic_d_t daemon_cx;
Hardware* hw;

// pointers to important DART objects
simulation::World* world;
dynamics::SkeletonDynamics* robot;

// workspace stuff
WorkspaceControl* ws;

/* ********************************************************************************************* */
/// Figures out a workspace velocity xdot that will take us from our current configuration to our 
/// reference configuration t_ref
void WorkspaceControl::getXdotFromXref(VectorXd& xdot) {

	// Get the current end-effector transform and also, just its orientation 
	Matrix4d Tcur = this->endeff->getWorldTransform();
	if(myDebug) DISPLAY_MATRIX(Tcur);
	Matrix4d Rcur = Tcur;
	Rcur.topRightCorner<3,1>().setZero();

	// Apply the similarity transform to the displacement between current transform and reference
	Matrix4d Tdisp = Tcur.inverse() * this->t_ref;
	Matrix4d xdotM = Rcur * Tdisp * Rcur.inverse();
	xdot = transformToEuler(xdotM, math::XYZ) * this->K_workerr_p;
}

/* ********************************************************************************************* */
// Use a dampened inverse Jacobian to compute jointspace velocities
// qdot from a given workspace velocity xdot. Uses nullspace
// projection to achieve a secondary goal. Important constants are for
// the null space gain, the dampening gain and the second goal pos.
void WorkspaceControl::getQdotFromXdot(const VectorXd& xdot, const VectorXd& qdot_nullspace, VectorXd& qdot) {

	// Get the Jacobian for the end-effector
	MatrixXd Jlin = this->endeff->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = this->endeff->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian with dampening
	MatrixXd Jt = J.transpose();
	MatrixXd JJt = J * Jt;
	for(int i = 0; i < JJt.rows(); i++) JJt(i,i) += this->damping_gain;
	
	// Compute the joint velocities qdot using the input xdot and a
	// qdot for the secondary goal projected into the nullspace
	MatrixXd Jinv = Jt * JJt.inverse();
	MatrixXd JinvJ = Jinv*J;
	MatrixXd I = MatrixXd::Identity(7,7);
	qdot = Jinv * xdot + (I - JinvJ) * qdot_nullspace * this->nullspace_gain;
}

/* ********************************************************************************************* */
// Returns the 6 axes values from the spacenav
bool WorkspaceControl::getSpaceNav(Eigen::VectorXd& config) {
	// Get joystick data
	int r = 0;
	config = Eigen::VectorXd::Zero(6);
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK(r, somatic__joystick,
			&protobuf_c_system_allocator, 4096, &this->spacenav_chan);
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return false;

	// Set the data to the input reference
	Somatic__Vector* x = js_msg->axes;
	config << -x->data[1], -x->data[0], -x->data[2], -x->data[4], -x->data[3], -x->data[5];

	// Free the liberty message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
	return true;
}

/* ********************************************************************************************* */
// moves the reference position by the given amount in the given direction
void WorkspaceControl::updateReferenceFromXdotInput(const VectorXd& xdot, const double dt) {

	// Represent the workspace velocity input as a 4x4 homogeneous matrix
	Matrix4d xdotM = eulerToTransform(xdot * dt, math::XYZ);
	
	// Compute the displacement in the end-effector frame with a similarity transform
	Matrix4d R = t_ref;
	R.topRightCorner<3,1>().setZero();
	Matrix4d Tdisp = R.inverse() * xdotM * R;

	// Update the reference position for the end-effector with the given workspace velocity
	this->t_ref = this->t_ref * Tdisp;
}

/* ********************************************************************************************* */
/// softly resist movements into joint limits. Basically just set a
/// velocity away from the limit and increase it as the joint gets
/// closer to the limit; eventually the system will reach a point
/// where the commanded velocity matches the velocity pushing back
/// and the joint will safely stop.
void computeQdotAvoidLimits(const Eigen::VectorXd& q, Eigen::VectorXd& qdot_avoid) {
	double error;
	double region;
	
	for(int i = 0; i < 7; i++) {
		// find our dof so we can get limits
		kinematics::Dof* dof = robot->getDof(left_arm_ids[i]);

		// store this so we aren't typing so much junk
		region = JOINTLIMIT_REGIONSIZE[i];
		// if(myDebug) printf("%d: min/max: (%lf/%lf), region: %lf\n", i, dof->getMin(), dof->getMax(), region);

		// no avoidance if we arne't near a limit
		qdot_avoid[i] = 0.0;

		// if we're close to hitting the lower limit, move positive
		if (q[i] < dof->getMin()) {
			qdot_avoid[i] = JOINTLIMIT_MAXVEL;
		}
		if (q[i] < dof->getMin() + region) {
			// figure out how close we are to the limit.
			error = q[i] - dof->getMin();

			// take the inverse to get the magnitude of our response
			// subtract 1/region so that our response starts at zero
			qdot_avoid[i] = (JOINTLIMIT_GAIN / error) - (JOINTLIMIT_GAIN / region);

			// clamp it to the maximum allowed avoidance strength
			qdot_avoid[i] = std::min(qdot_avoid[i], JOINTLIMIT_MAXVEL);
		}

		// if we're close to hitting the lower limit, move positive
		if (q[i] > dof->getMax()) {
			qdot_avoid[i] = -JOINTLIMIT_MAXVEL;
		}
		if (q[i] > dof->getMax() - region) {
			// figure out how close we are to the limit.
			error = q[i] - dof->getMax();

			// take the inverse to get the magnitude of our response
			// subtract 1/region so that our response starts at zero
			qdot_avoid[i] = (JOINTLIMIT_GAIN / error) + (JOINTLIMIT_GAIN / region);

			// clamp it to the maximum allowed avoidance strength
			qdot_avoid[i] = std::max(qdot_avoid[i], -JOINTLIMIT_MAXVEL);
		}
	}
}

/* ******************************************************************************************** */
/// Clean up
void destroy() {

	// Stop the daemon
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
		SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Clean up the workspace stuff, just to be safe
	delete ws;

	// Close down the hardware
	printf("closing the hardware\n");
	delete hw;
	printf("closed the hardware\n"); fflush(stdout);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	// Print the z input data
	// for(size_t i = 0; i < data.size(); i++) printf("%lf\n", data[i]);
}

/* ********************************************************************************************* */
/// The main loop
void run() {

	// Set the constants
	const double SPACENAV_ORIENTATION_GAIN = 0.25;
	const double SPACENAV_TRANSLATION_GAIN = 0.25; 
	const double COMPLIANCE_GAIN = 1.0 / 750.0;

	// Get the last time to compute the passed time
	int c_ = 0, badSpaceNavCtr = 0;
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	Eigen::VectorXd last_spacenav_input = Eigen::VectorXd::Zero(6);
	while(!somatic_sig_received) {

		myDebug = ((c_++ % 30) == 1);

		// Update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// Pretty output
		if(myDebug) std::cout << "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << std::endl;

		// Update the robot
		hw->updateSensors(time_delta);
		if(myDebug) hw->printState();

		// Check for too high currents
		const double CURRENT_WARN_LIMIT = 10.0, CURRENT_KILL_LIMIT = 12.0;
		if(myDebug) DISPLAY_VECTOR(eig7(hw->larm->cur));
		for(size_t i = 0; i < 7; i++) {
			if(fabs(hw->larm->cur[i]) > CURRENT_WARN_LIMIT)
				printf("\t\t\tWARNING: Current at module %zu has passed %lf amps: %lf amps\n", i, 	
					CURRENT_WARN_LIMIT, hw->larm->cur[i]);
			if(fabs(hw->larm->cur[i]) > CURRENT_KILL_LIMIT) {
				printf("\t\t\tStopping because current at module %zu has passed %lf amps: %lf amps\n", i,
					 CURRENT_KILL_LIMIT, hw->larm->cur[i]);
				destroy();
				exit(0);
			}
		}

		// Update the spacenav
		Eigen::VectorXd spacenav_input;
		if(!ws->getSpaceNav(spacenav_input)) spacenav_input = (badSpaceNavCtr++ > 20) ? VectorXd::Zero(6) : last_spacenav_input;
		else badSpaceNavCtr = 0;
		last_spacenav_input = spacenav_input;

		// Scale the spacenav input to get a workspace velocity 
		Eigen::VectorXd xdot_spacenav = spacenav_input;
		xdot_spacenav.topLeftCorner<3,1>() *= SPACENAV_TRANSLATION_GAIN;
		xdot_spacenav.bottomLeftCorner<3,1>() *= SPACENAV_ORIENTATION_GAIN;
		if(myDebug) DISPLAY_VECTOR(xdot_spacenav);

		// Move the workspace references around from that spacenav input
		ws->updateReferenceFromXdotInput(xdot_spacenav, time_delta);
		if(myDebug) DISPLAY_MATRIX(ws->t_ref);

		// Compute an xdot for complying with external forces if the f/t values are within thresholds
		Eigen::VectorXd xdot_comply;
		if(myDebug) DISPLAY_VECTOR(hw->lft->lastExternal);
		xdot_comply = -hw->lft->lastExternal * COMPLIANCE_GAIN;
		if(myDebug) DISPLAY_VECTOR(xdot_comply);

		// Get an xdot out of the P-controller that's trying to drive us to the refernece position
		Eigen::VectorXd xdot_posref;
		ws->getXdotFromXref(xdot_posref);
		if(myDebug) DISPLAY_VECTOR(xdot_posref);

		// Combine the velocities from the workspace position goal, the spacenav, and the compliance
		Eigen::VectorXd xdot_apply = xdot_posref + xdot_spacenav + xdot_comply;
		if(myDebug) DISPLAY_VECTOR(xdot_apply);

		// Compute qdot for our secondary goal (currently, drive qo to
		// a reference position
		Eigen::VectorXd q = robot->getConfig(*ws->arm_ids);
		Eigen::VectorXd qdot_secondary = q - ws->nullspace_q_ref;

		// Compute qdot_jacobian with the dampened inverse Jacobian,
		// using nullspace projection to achieve our secondary goal
		Eigen::VectorXd qdot_jacobian;
		ws->getQdotFromXdot(xdot_apply, qdot_secondary, qdot_jacobian);

		// Scale the values down if the norm is too big or set it to zero if too small
		double magn = qdot_jacobian.norm();
		if(magn > 0.5) qdot_jacobian *= (0.5 / magn);
		else if(magn < 0.05) qdot_jacobian = VectorXd::Zero(7);
		if(myDebug) DISPLAY_VECTOR(qdot_jacobian);

		// avoid joint limits - set velocities away from them as we get close
		Eigen::VectorXd qdot_avoid(7);
		if(myDebug) DISPLAY_VECTOR(q);
		computeQdotAvoidLimits(q, qdot_avoid);
		if(myDebug) DISPLAY_VECTOR(qdot_avoid);

		// add our qdots together to get the overall movement
		Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;
		if(myDebug) DISPLAY_VECTOR(qdot_apply);

		// Send the velocity command
		somatic_motor_setvel(&daemon_cx, hw->larm, qdot_apply.data(), 7);

		// and sleep to fill out the loop period so we don't eat the
		// entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		if(myDebug) std::cout << "sleeping for: " << time_sleep_usec << std::endl;
		usleep(time_sleep_usec);
		
/*
		if (myDebug && time_sleep < 0.0) {
			std::cout << "Taking too long! Took " << std::fixed << std::setw(8) << time_delta
			          << "/" << std::fixed << std::setw(8) << 1.0/LOOP_FREQUENCY << " seconds" << std::endl;
		}
*/
	}
}

/* ******************************************************************************************** */
/// Initialization
void init() {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// Initialize the daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "05-workspace";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Hardware::Mode mode = (Hardware::Mode)((Hardware::MODE_ALL & ~Hardware::MODE_RARM) & ~Hardware::MODE_GRIPPERS);
	hw = new Hardware(mode, &daemon_cx, robot);

	// Set up the workspace controllers
	ws = new WorkspaceControl(&daemon_cx, "spacenav-data", &left_arm_ids, robot->getNode("lGripper"));

	// Start the daemon running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
		SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ******************************************************************************************** */
WorkspaceControl::WorkspaceControl(somatic_d_t* _daemon_cx,
                                   const char* _spacenav_chan_name,
                                   std::vector<int>* _arm_ids,
                                   kinematics::BodyNode* _endeff,
                                   double _K_workerr_p) {
	// grab variables we want to hold on to
	this->daemon_cx = _daemon_cx;
	this->endeff = _endeff;
	this->K_workerr_p = _K_workerr_p;
	this->arm_ids = _arm_ids;

	// open up our ach channel
	somatic_d_channel_open(this->daemon_cx, &this->spacenav_chan, _spacenav_chan_name, NULL);

	// initialize our position reference to the end effector's current
	// position, just to be safe.
	this->t_ref = this->endeff->getWorldTransform();

	// Set some constants
	this->K_workerr_p = K_WORKERR_P_INIT;
	this->nullspace_gain = NULLSPACE_GAIN_INIT;
	this->nullspace_q_ref = NULLSPACE_Q_REF_INIT;
	this->damping_gain = DAMPING_GAIN_INIT;
}

/* ******************************************************************************************** */
/// Closes the spacenav channel
WorkspaceControl::~WorkspaceControl() {
	somatic_d_channel_close(this->daemon_cx, &this->spacenav_chan);
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {
	init();
	run();
	destroy();
}
