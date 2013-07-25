/**
 * @file 05-workspace.cpp
 * @author Saul Reynolds-Haertle
 * @date July 25, 2013
 * @brief This file demonstrates workspace control running on
 * Krang. It will probably be a workhorse for general tasks, including
 * demonstrations.
 */

// #############################################################################
// #############################################################################
// ### Includes
// #############################################################################
// #############################################################################

#include "kore.h"

#include <iostream>
#include <iomanip>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <math/UtilsRotation.h>
#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/BodyNode.h>

#include "util.h"

// #############################################################################
// #############################################################################
// ### Constants
// #############################################################################
// #############################################################################

const char* DAEMON_IDENT = "05-workspace";

// home configuration for the arms
// TODO: figure out right arm home config
const Eigen::VectorXd HOMECONFIGL = (VectorXd(7) << 1.102, -0.589, 0.000, -1.339, 0.000, -0.959, -1.000).finished();
const Eigen::VectorXd HOMECONFIGR = (VectorXd(7) << -1.102, 0.589, 0.000, 1.339, 0.141, 0.959, -1.000).finished();

// initializers for the workspace control constants
const double K_WORKERR_P_INIT = 0.005;
const double K_NULLSPACE_P_INIT = 0.0;
const Eigen::VectorXd NULLSPACE_Q_REF_INIT = (VectorXd(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
const double DAMPING_GAIN_INIT = 0.005;

// user interface constants
const double SPACENAV_ORIENTATION_GAIN = 3.0;
const double SPACENAV_TRANSLATION_GAIN = 1.0;

// loop frequency - be nice to other people on the CPU.
const double LOOP_FREQUENCY = 300.0;

// #############################################################################
// #############################################################################
// ### Type declarations
// #############################################################################
// #############################################################################

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
	void updateReferenceFromXdot(const VectorXd& xdot, const double dt);
	void getXdotFromXref(VectorXd& xdot);
	void getQdotFromXdot(const VectorXd& xdot, VectorXd& qdot);

	// member variables
	Eigen::Matrix4d t_ref;
	ach_channel_t spacenav_chan;
	somatic_d_t* daemon_cx;
	kinematics::BodyNode* endeff;
	std::vector<int>* arm_ids;

	// configuration stuff
	double K_workerr_p;        // p constant for the PID controller that produces workspace movements
	double K_nullspace_p;      // p constant for the PID controller that pulls the arm through nullspace to the secondary goal
	VectorXd nullspace_q_ref;  // secondary goal configuration which we use the nullspace to move toward
	double damping_gain;       // how powerful the damping is on our damped jacobian
};

// #############################################################################
// #############################################################################
// ### Variable declarations
// #############################################################################
// #############################################################################

// state variables for the daemon - one daemon context and one
// hardware object
somatic_d_t daemon_cx;
Krang::Hardware* h;

// pointers to important DART objects
simulation::World* world;
dynamics::SkeletonDynamics* robot;

// workspace stuff
std::map<Krang::Side, WorkspaceControl*> arm_ws;

// #############################################################################
// #############################################################################
// ### Function declarations
// #############################################################################
// #############################################################################

// #############################################################################
// #############################################################################
// ### Helpers
// #############################################################################
// #############################################################################

// tricked-out debug print for collections. Prints fixed-point,
// fixed-width, and fixed-precision for perfectly aligned columns and
// no annoying exponents.
#define DISPLAY_VECTOR(VEC) \
	std::cout << std::setw(25) << std::left << #VEC; \
	for(int i = 0; i < VEC.size(); i++) std::cout << std::fixed << std::setw(12) << VEC[i]; \
	std::cout << std::endl;

// similar debug print for matrices
#define DISPLAY_MATRIX(MAT) \
	std::cout << std::setw(25) << std::left << #MAT << std::endl; \
	for(int c = 0; c < MAT.cols(); c++) { \
	std::cout << "    "; \
for(int r = 0; r < MAT.rows(); r++) {	  \
	std::cout << std::fixed << std::setw(12) << MAT(r, c); } \
std::cout << std::endl; \
}	\
std::cout << std::endl;

	

// figures out a workspace velocity xdot that will take us from our
// current configuration to our reference configuration t_ref
void WorkspaceControl::getXdotFromXref(VectorXd& xdot) {
	// Get the current end-effector transform and also, just its orientation 
	Matrix4d Tcur = this->endeff->getWorldTransform();
	Matrix4d Rcur = Tcur;
	Rcur.topRightCorner<3,1>().setZero();

	// Apply the similarity transform to the displacement between current transform and reference
	Matrix4d Tdisp = Tcur.inverse() * this->t_ref;
	Matrix4d xdotM = Rcur * Tdisp * Rcur.inverse();
	xdot = transformToEuler(xdotM, math::XYZ) * this->K_workerr_p;
}

// Use a dampened inverse Jacobian to compute jointspace velocities
// qdot from a given workspace velocity xdot. Uses nullspace
// projection to achieve a secondary goal. Important constants are for
// the null space gain, the dampening gain and the second goal pos.
void WorkspaceControl::getQdotFromXdot(const VectorXd& xdot, VectorXd& qdot) {
	// Get the Jacobian for the left end-effector
	MatrixXd Jlin = this->endeff->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = this->endeff->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian with dampening
	MatrixXd Jt = J.transpose(), JJt = J * Jt;
	for (int i=0; i < JJt.rows(); i++) JJt(i,i) += this->damping_gain;

	// Compute joint velocities that will achieve our secondary goal;
	// we will project this into the nullspace to it doesn't affect
	// the primary goal.
	VectorXd q = robot->getConfig(*this->arm_ids);
	VectorXd qDotRef = (q - this->nullspace_q_ref) * this->K_nullspace_p;
	
	// Compute the joint velocities qdot using the input xdot and a
	// qdot for the secondary goal projected into the nullspace
	MatrixXd Jinv = Jt * JJt.inverse();
	MatrixXd JinvJ = Jinv*J;
	MatrixXd I = MatrixXd::Identity(7,7);
	qdot = Jinv * xdot + (I - JinvJ) * qDotRef;
}

// #############################################################################
// #############################################################################
// ### Update
// #############################################################################
// #############################################################################

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

// moves the reference position by the given amount in the given direction
void WorkspaceControl::updateReferenceFromXdot(const VectorXd& xdot, const double dt) {
	// turn our input into a matrix
	Matrix4d xdotM = eulerToTransform(xdot * dt, math::XYZ);
	
	// Compute the displacement in the end-effector frame with a similarity transform
	Matrix4d R = t_ref;
	R.topRightCorner<3,1>().setZero();
	Matrix4d Tdisp = R.inverse() * xdotM * R;

	// Update the reference position for the end-effector with the given workspace velocity
	this->t_ref = this->t_ref * Tdisp;
}

// #############################################################################
// #############################################################################
// ### Main loop
// #############################################################################
// #############################################################################

void run() {
	// timing stuff
	double time_last = aa_tm_timespec2sec(aa_tm_now());

	while(!somatic_sig_received) {
		// update times
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;

		// pretty output
		std::cout << "################################################################################" << std::endl;

		// update the robot
		h->updateSensors(time_delta);
		h->updateKinematics();

		// update the spacenav
		Eigen::VectorXd spacenav_input_L;
		arm_ws[Krang::LEFT]->getSpaceNav(spacenav_input_L);
		spacenav_input_L.topLeftCorner<3,1>() *= SPACENAV_TRANSLATION_GAIN;
		spacenav_input_L.bottomLeftCorner<3,1>() *= SPACENAV_ORIENTATION_GAIN;
		DISPLAY_VECTOR(spacenav_input_L);

		// Eigen::VectorXd spacenav_input_R;
		// arm_ws[Krang::RIGHT]->getSpaceNav(spacenav_input_R);
		// spacenav_input_R.topLeftCorner<3,1>() *= SPACENAV_TRANSLATION_GAIN;
		// spacenav_input_R.bottomLeftCorner<3,1>() *= SPACENAV_ORIENTATION_GAIN;
		// DISPLAY_VECTOR(spacenav_input_R);
		
		// move the workspace references around from that spacenav input
		arm_ws[Krang::LEFT]->updateReferenceFromXdot(spacenav_input_L, time_delta);
		DISPLAY_MATRIX(arm_ws[Krang::LEFT]->t_ref);

		// arm_ws[Krang::RIGHT]->updateReferenceFromXdot(spacenav_input_R, time_delta);
		// DISPLAY_MATRIX(arm_ws[Krang::RIGHT]->t_ref);

		// get an xdot out of the PID controller that's trying to
		// drive us to the refernece position
		Eigen::VectorXd xdotL;
		arm_ws[Krang::LEFT]->getXdotFromXref(xdotL);
		DISPLAY_VECTOR(xdotL);

		// Eigen::VectorXd xdotR;
		// arm_ws[Krang::RIGHT]->getXdotFromXref(xdotR);
		// DISPLAY_VECTOR(xdotR);

		// turn that xdot into a qdot using our jacobian stuff
		Eigen::VectorXd qdotL;
		arm_ws[Krang::LEFT]->getQdotFromXdot(xdotL, qdotL);
		DISPLAY_VECTOR(qdotL);

		// Eigen::VectorXd qdotR;
		// arm_ws[Krang::LEFT]->getQdotFromXdot(xdotL, qdotR);
		// DISPLAY_VECTOR(qdotR);

		// send the qdot
		// somatic_motor_setvel(&daemon_cx, h->larm, qdotL, 7);
		// somatic_motor_setvel(&daemon_cx, h->rarm, qdotR, 7);

		// and sleep to fill out the loop period so we don't eat the
		// entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		aa_tm_relsleep(aa_tm_sec2timespec(time_sleep));
	}
}

// #############################################################################
// #############################################################################
// ### Startup and shutdown
// #############################################################################
// #############################################################################

void init() {
	// init dart. do this before we initialize the daemon because
	// initing the daemon changes our current directory to somewhere
	// in /var/run.
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// init daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = DAEMON_IDENT;
	somatic_d_init(&daemon_cx, &daemon_opt);

	// init hardware
	std::cout << "opening hardware" << std::endl;
	h = new Krang::Hardware((Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL & ~Krang::Hardware::MODE_GRIPPERS), &daemon_cx, robot);
	std::cout << "opened hardware" << std::endl;

	// set up the workspace controllers
	arm_ws[Krang::LEFT] = new WorkspaceControl(&daemon_cx, "spacenav-data-l",
	                                    &Krang::left_arm_ids, robot->getNode("lGripper"));
	// arm_ws[Krang::RIGHT] = new WorkspaceControl(&daemon_cx, "spacenav-data-r",
	//                                      &Krang::right_arm_ids, robot->getNode("rGripper"));

	// start the daemon running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

void destroy() {
	// stop the daemon
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// clean up the workspace stuff, just to be safe
	delete arm_ws[Krang::LEFT];
	delete arm_ws[Krang::RIGHT];
	arm_ws.clear();

	// close down the hardware
	delete h;

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);
}

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
	this->K_nullspace_p = K_NULLSPACE_P_INIT;
	this->nullspace_q_ref = NULLSPACE_Q_REF_INIT;
	this->damping_gain = DAMPING_GAIN_INIT;
}

WorkspaceControl::~WorkspaceControl() {
	// close down our ach channel
	somatic_d_channel_close(this->daemon_cx, &this->spacenav_chan);
}

// #############################################################################
// #############################################################################
// ### Main
// #############################################################################
// #############################################################################

int main(int argc, char* argv[]) {
	init();
	run();
	destroy();
}
