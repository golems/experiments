/**
 * @file 03-workspace.cpp
 * @author Jon Scholz
 * @date July 12, 2013
 * @brief This file demonstrates how to visualize the motion of the arms in grip,
 * under workspace control from a joystick or liberty UI device
 */

#define protected public
#define private public

#include "simTab.h"
#include "GRIPApp.h"

#include "channel.h"
#include <math/UtilsRotation.h>

#include "util.h"
#include "KrangControl.h"
#include "WorkspaceControl.h"
#include "SpacenavTeleop.h"

#include "liberty_client.h"
#include "joystick_client.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;

/* ********************************************************************************************* */

// UI teleop objects
SpacenavTeleop spn1;
SpacenavTeleop spn2;

// Krang the monster
KrangControl krang;

// UI Globals
/* Events */
enum DynamicSimulationTabEvents {
	id_button_ResetScene = 8100,
	id_button_ResetLiberty,
	id_button_ResetJoystick,
	id_button_SetInitialTransforms,
	id_checkbox_ToggleMotorInputMode,
	id_checkbox_ToggleMotorOutputMode,
	id_checkbox_ToggleRightTrackLeftMode,
};

// control globals
enum UI_MODES {
	UI_LIBERTY = 0,
	UI_JOYSTICK,
	UI_MIXED,
};
static bool motor_input_mode = 0;
static bool motor_output_mode = 0;
static bool motors_initialized = 0;
static bool right_track_left_mode = 0;
static int ui_input_mode = UI_JOYSTICK;

// Pointers to frequently used dart data structures
kinematics::BodyNode *eeNodeL;
kinematics::BodyNode *eeNodeR;
dynamics::SkeletonDynamics *goalSkelL;
dynamics::SkeletonDynamics *goalSkelR;

// Effector and liberty transforms
MatrixXd T_eeL_init; 	//< left effector initial transform
MatrixXd T_eeL_cur; 	//< left effector global transform

MatrixXd T_eeR_init; 	//< right effector initial transform
MatrixXd T_eeR_cur; 	//< right effector global transform

MatrixXd T_lib1_init; 	//< liberty channel1 initial transform
MatrixXd T_lib1_cur; 	//< liberty channel1 global transform

MatrixXd T_lib2_init; 	//< liberty channel2 transform
MatrixXd T_lib2_cur; 	//< liberty channel2 global transform

MatrixXd T_eeL_goal; 	//< liberty channel1 global transform
MatrixXd T_eeR_goal; 	//< liberty channel2 global transform

//MatrixXd T_joy_init;	//< joystick device initial transform
//MatrixXd T_joy_cur;		//< joystick device current transform

MatrixXd T_eeL_eeR; //< left effector transform in right effector frame
MatrixXd T_eeR_eeL; //< right effector transform in left effector frame

MatrixXd T_dummy = MatrixXd(4,4).setIdentity(4,4); // dummy transform for 1-arm control

/************************ Initialization **************************/

void initialize_robot() {
	initArm(daemon_cx, llwa, "llwa");
	initArm(daemon_cx, rlwa, "rlwa");
	initWaist(daemon_cx, waist);
	initIMU(daemon_cx, imu_chan);
	ach_open(&lgripper_chan, "lgripper-cmd", NULL);
	ach_open(&rgripper_chan, "rgripper-cmd", NULL);
	motors_initialized = 1;
}

/*
 * Caches all relevant transforms from the robot's current state
 */
void initialize_transforms(simulation::World* world) {
//	T_joy_init = Matrix4d(4,4); //< initial joystick transform
//	T_joy_cur = Matrix4d(4,4); //< current joystick transform

	T_eeL_init = Matrix4d(4,4); //< left effector global transform
	T_eeR_init = Matrix4d(4,4); //< right effector global transform
	T_lib1_init = Matrix4d(4,4); //< liberty channel1 global transform
	T_lib2_init = Matrix4d(4,4); //< liberty channel2 global transform

	T_eeL_cur = Matrix4d(4,4); //< left effector global transform
	T_eeR_cur = Matrix4d(4,4); //< right effector global transform
	T_lib1_cur = Matrix4d(4,4); //< liberty channel1 global transform
	T_lib2_cur = Matrix4d(4,4); //< liberty channel2 global transform

	T_eeL_goal = Matrix4d(4,4); //< left effector goal transform
	T_eeR_goal = Matrix4d(4,4); //< left effector global transform

	T_eeL_eeR = Matrix4d(4,4); ///< left effector transform in right effector frame
	T_eeR_eeL = Matrix4d(4,4); ///< right effector transform in left effector frame

	// grab pose of liberty channels 1 and 2
	MatrixXd *Tlibs[] = {&T_lib1_init, &T_lib2_init};
	getLibertyPoses(Tlibs, 2, NULL);

	// grab joystick pose
//	getJoystickPose(T_joy_init);

	// grab pose of robot left and right effectors
	T_eeL_init = eeNodeL->getWorldTransform();
	T_eeR_init = eeNodeR->getWorldTransform();

	// set the effector goal poses to their current states
	T_eeL_goal = T_eeL_init;
	T_eeR_goal = T_eeR_init;

	// cache the relative effector transfomrs
	T_eeL_eeR = T_eeR_init.inverse() * T_eeL_init;
	T_eeR_eeL = T_eeL_init.inverse() * T_eeR_init;
}

/****************************************************************************************
 * Workspace goal update functions
 ****************************************************************************************/

/*
 * Reads the current spacenav joystick channel values, and uses them to update
 * the goal transform for the given effector.
 * Spacenav values are interpreted as VELOCITIES in the WORLD FRAME, which are
 * added to the current effector pose (using quaternions for orientation)
 */
void updateGoalFromSpnavVels(kinematics::BodyNode *eeNode, MatrixXd& goalTransform,
		SpacenavTeleop &spn, double scale, dynamics::SkeletonDynamics *goalSkel = NULL) {

	/*
	 * TODO:
	 * 1) un-reverse spacenav pitch dimension
	 * 2) get jacobian for pitch dimension to work with eulerToTransform
	 * 3) drift thing
	 * ...
	 * > get rid of normalize crap
	 * > add damping and nullspace projection
	 */

	// Get the config of the spacenav
	VectorXd goalConfig(6);
	spn.getPose(goalTransform, &goalConfig);


//cout << "\ngoalConfig raw: " << goalConfig.transpose() << endl;
//cout << "goalTransform: " << goalTransform << endl;
//cout << "goalConfig ftr: " << transformToEuler(goalTransform, math::XYZ).transpose() << endl;

	// scale workspace velocity dims
	double weights[] = {1, 1, 1, 2, 2, 2};
	for (int i=0; i < 6; ++i)
		goalConfig[i] *= weights[i];

	// Set the location around the current end-effector pose with some scale
	Matrix4d eeT = eeNode->getWorldTransform();
	goalConfig.topLeftCorner<3,1>() = eeT.topRightCorner<3,1>() + scale * goalConfig.topLeftCorner<3,1>();

	// Set the orientation "change" with respect to the end-effector orientation
	Quaternion<double> goalOri(goalTransform.topLeftCorner<3,3>());
	Quaternion <double> eeOri(eeT.topLeftCorner<3,3>());
	Quaternion<double> errOriQ = goalOri * eeOri.inverse();
	Matrix3d errOriM = errOriQ.matrix();
	goalConfig.bottomLeftCorner<3,1>() = math::matrixToEuler(errOriM, math::XYZ);

	// update goalTransform from config representation
	goalTransform = eulerToTransform(goalConfig, math::XYZ);

	// Update the dart data structure and set the goal
	if (goalSkel != NULL) {
		goalSkel->setConfig(dartRootDofOrdering, goalConfig);
		//goalTransform = goalSkel->getNode("link_0")->getWorldTransform(); //TODO deleteme!
	}
}

/****************************************************************************************
 * Jacobian velocity functions
 ****************************************************************************************/
/*
 * Computes desired workspace effector velocities xdot based on the
 * provided goals, and returns jointspace velocities qdot from jacobian
 */
VectorXd computeWorkVelocity(const kinematics::BodyNode* eeNode, const MatrixXd &eeGoal) {

	VectorXd xdot(6); xdot.setZero(6);

	// Get the current goal location and orientation as a quaternion
	VectorXd goalPos = eeGoal.topRightCorner<3,1>();
	Quaternion<double> goalOri(eeGoal.topLeftCorner<3,3>());

	// Get the current end-effector location and orientation as a quaternion;
	MatrixXd eeTransform = eeNode->getWorldTransform();
	VectorXd eePos = eeTransform.topRightCorner<3,1>();
	Quaternion <double> eeOri(eeTransform.topLeftCorner<3,3>());

	// Find the position error
	VectorXd errPos = goalPos - eePos;

	// Find the orientation error and express it in RPY representation
	Quaternion<double> errOriQ = goalOri * eeOri.inverse();
	Matrix3d errOriM = errOriQ.matrix();
	Vector3d errOriV = math::matrixToEuler(errOriM, math::XYZ);

	// Get the workspace velocity
	//errOriV= Vector3d(0.0, 0.0, 0.0);
	xdot << errPos, errOriV;
	return xdot;
}

/*
 * Given a workspace velocity, returns the joint space velocity
 */
VectorXd workToJointVelocity (kinematics::BodyNode* eeNode, const VectorXd& xdot,
		double xdotGain, double nullGain, VectorXd *q = NULL) {

	// Get the Jacobian towards computing joint-space velocities
	MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jsq = J * Jt;
	for (int i=0; i < Jsq.rows(); i++)
		Jsq(i,i) += 0.005;
	Eigen::MatrixXd Jinv = Jt * Jsq.inverse();

	// Get the joint-space velocities by multiplying inverse Jacobian with x (simple version)
	//VectorXd qdot = Jinv * xdot;

    /*
     * Do the null-space projection thing to bias our solution towards
     * joint values in the middle of each joint's range of motion
     *
     * scale must be negative to move towards rather than away
     */

	// Compute Joint Distance from middle of range
	VectorXd qDist(7); qDist.setZero(7);
	if (q != NULL)
		qDist = q->cwiseAbs();

	MatrixXd JinvJ = Jinv*J;
	MatrixXd I = MatrixXd::Identity(7,7);
	VectorXd qdot = Jinv * (xdot * xdotGain) + (I - JinvJ) * (qDist * nullGain);

	return qdot;
}

/****************************************************************************************
 * Helper functions for Grip-only visualization
 ****************************************************************************************/
/*
 * Given the two goal 4x4 transformation for the arrows, draw them
 */
void setGoalSkelConfig(dynamics::SkeletonDynamics *goalSkel, const MatrixXd &goal) {
	// convert goal to euler
	VectorXd arrowConf = transformToEuler(goal, math::XYZ);

	// set config of skeleton
	goalSkel->setConfig(dartRootDofOrdering, arrowConf);
}

/*
 * Manually updates the arm configuration for the provided jointspace
 * velocities, scaled by the provided dt
 * TODO use dart dynamics
 */
void fakeArmMovement(vector<int> ids, VectorXd &qdot, double dt) {

	VectorXd q = mWorld->getSkeleton("Krang")->getConfig(ids);
	q += qdot * dt;
	mWorld->getSkeleton("Krang")->setConfig(ids, q);
}


/****************************************************************************************
 * Primary arm control function
 ****************************************************************************************/
void updateArm(kinematics::BodyNode *eeNode, MatrixXd &eeGoal, dynamics::SkeletonDynamics *goalSkel,
		somatic_motor_t &arm,bool motor_output_mode, vector<int> armIDs, double dt) {

	// Compute workspace velocity from goal errors
	VectorXd xdot = computeWorkVelocity(eeNode, eeGoal);

	// Convert to jointspace velocities
	VectorXd q = mWorld->getSkeleton("Krang")->getConfig(armIDs);
	VectorXd qdot = workToJointVelocity(eeNode, xdot, 5.0, 0.01, &q);

	// dispatch joint velocities to arms
	if (motor_output_mode) {
		// send arm velocities
		double extraBoost = 4.0;
		sendRobotArmVelocities(daemon_cx, arm, qdot, dt * extraBoost);

	} else {
		// Move the arms with a small delta using the computed velocities

		if(xdot.norm() > 1e-2)
			fakeArmMovement(armIDs, qdot, dt);
	}
}

/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {

	// update robot state from ach if we're controlling the actual robot
	if (motor_input_mode) {
//		updateRobotSkelFromSomaticMotor(mWorld, daemon_cx, llwa, armIDsL);
//		updateRobotSkelFromSomaticMotor(mWorld, daemon_cx, rlwa, armIDsR);
//		updateRobotSkelFromSomaticWaist(mWorld, daemon_cx, waist, waistIDs);
//		updateRobotSkelFromIMU(mWorld);
		krang.updateKrangSkeleton(mWorld);
	}

	// Get the goal configuration from spacenav
	updateGoalFromSpnavVels(eeNodeL, T_eeL_goal, spn1, 0.2, goalSkelL);

	if (right_track_left_mode) {
		// hack: get right arm goal by tracking left arm
		T_eeL_cur = eeNodeL->getWorldTransform();
		T_eeR_goal = T_eeL_cur * T_eeR_eeL;
		VectorXd goalConfigR = transformToEuler(T_eeR_goal, math::ZYX);
		goalSkelR->setConfig(dartRootDofOrdering, goalConfigR);
	} else {
		updateGoalFromSpnavVels(eeNodeR, T_eeR_goal, spn2, 0.2, goalSkelR);
	}

	// handle grippers
//	VectorXi buttons = getSpacenavButtons(spn_chan);
//	cout << "buttonsL "<< buttons.transpose() << endl;
//	handleSpacenavButtons(buttons, rqd_chan);

	// Compute timestep
	static double last_movement_time = aa_tm_timespec2sec(aa_tm_now());
	double current_time = aa_tm_timespec2sec(aa_tm_now());
	double dt = current_time - last_movement_time;
	last_movement_time = current_time;

	updateArm(eeNodeL, T_eeL_goal, goalSkelL, llwa, motor_output_mode, armIDsL, dt);
	updateArm(eeNodeR, T_eeR_goal, goalSkelR, rlwa, motor_output_mode, armIDsR, dt);

	// Visualize the arm motion
	viewer->DrawGLScene();
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
		long style) : GRIPTab(parent, id, pos, size, style) {
	// ============================================================================
	// Do basic tab layout
	// Create user interface
	wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("UI Input"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Initialization"));
	wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Control Output"));

	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
	wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

	ss1BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleMotorInputMode, wxT("Read Motor State")), 0, wxALL, 1);

	ss2BoxS->Add(new wxButton(this, id_button_ResetScene, wxT("Reset Scene")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetLiberty, wxT("Set Liberty Initial Transforms")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetJoystick, wxT("Set Joystick Initial Transform")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_SetInitialTransforms, wxT("Set Arm Initial Transforms")), 0, wxALL, 1);

	ss3BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleMotorOutputMode, wxT("Send Motor Commands")), 0, wxALL, 1);
	ss3BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleRightTrackLeftMode, wxT("Track left arm with right")), 0, wxALL, 1);

	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

	SetSizer(sizerFull);

	// ============================================================================
	// load hard-coded scene
	frame->DoLoad("../../common/scenes/04-World-Liberty.urdf");
	if (mWorld == NULL)
		frame->DoLoad("common/scenes/04-World-Liberty.urdf"); // for eclipse

	// ============================================================================
	// set viewer
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(0.0, 0.0, -0.7);
	viewer->UpdateCamera();

	// ============================================================================
	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->Start(0.005 * 1e4);

	// ============================================================================
	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "03-workspace";
	somatic_d_init(&daemon_cx, &dopt);

	// Set the interrupt handler
	somatic_sighandler_simple_install();

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);


	// set arm indices and configs
	setDartIDs(mWorld);

	// Manually set the initial arm configuration for the left arm
	VectorXd larm_conf(7), rarm_conf(7);
	larm_conf << 0.0, -M_PI / 3.0, 0.0, -M_PI / 3.0, 0.0, M_PI/6.0, 0.0;
	rarm_conf << 0.0, M_PI / 3.0, 0.0, M_PI / 3.0, 0.0, -M_PI/6.0, 0.0;
	mWorld->getSkeleton("Krang")->setConfig(armIDsL, larm_conf);
	mWorld->getSkeleton("Krang")->setConfig(armIDsR, rarm_conf);

	// Initialize the joystick(s)
//	somatic_d_channel_open(&daemon_cx, &spacenav_chan, "spacenav-data", NULL);
//	somatic_d_channel_open(&daemon_cx, &spacenav_chan2, "spacenav2-data", NULL);
	spn1.initialize(&daemon_cx, "spacenav-data");
	spn2.initialize(&daemon_cx, "spacenav2-data");

	// Initialize the liberty
	initLiberty();

	// Initialize the arms
	if (motor_input_mode || motor_output_mode) {
		//initialize_robot();
		// initialize krang the monster
		krang.initialize(&daemon_cx);
	}

	// Grab effector node pointer to compute the task space error and the Jacobian
	eeNodeL = mWorld->getSkeleton("Krang")->getNode("lGripper");
	eeNodeR = mWorld->getSkeleton("Krang")->getNode("rGripper");

	// Grab goal skeleton references
	goalSkelL = mWorld->getSkeleton("g1");
	goalSkelR = mWorld->getSkeleton("g2");

	// Initialize primary transforms
	usleep(1e5); // give ach time to initialize, or init transforms will be bogus
	initialize_transforms(mWorld);
}

/* ********************************************************************************************* */
SimTab::~SimTab() {

	// halt the arms TODO: why isn't this being called?
//	haltArm(daemon_cx, llwa);
//	haltArm(daemon_cx, rlwa);
	krang.halt();
	motors_initialized = 0;

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Clean up the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() {}

/**
 * @function OnButton
 * @brief Handles button events
 */
void SimTab::OnButton(wxCommandEvent &evt) {
	int slnum = evt.GetId();
	switch(slnum) {
	// Set Start Arm Configuration
	case id_button_ResetScene: {
		break;
	}

	case id_button_ResetLiberty: {
		MatrixXd *Tlibs[] = {&T_lib1_init, &T_lib2_init};
		getLibertyPoses(Tlibs, 2, NULL);
		break;
	}

	case id_button_ResetJoystick: {
//		getJoystickPose(T_joy_init);
		break;
		}

	case id_button_SetInitialTransforms: {
		// grab pose of robot left and right effectors
		//T_eeL_init = eeNodeL->getWorldTransform();
		//T_eeR_init = eeNodeR->getWorldTransform();
		initialize_transforms(mWorld);
		break;
	}

	case id_checkbox_ToggleMotorInputMode: {
		motor_input_mode = evt.IsChecked();
		if (motor_input_mode && !motors_initialized) {
			initialize_robot();
		}
		break;
	}

	case id_checkbox_ToggleMotorOutputMode: {

		motor_output_mode = evt.IsChecked();
		if (motor_output_mode && !motors_initialized) {
			initialize_robot();
		}

		if (!motor_output_mode) {
			haltArm(daemon_cx, llwa);
			haltArm(daemon_cx, rlwa);
			motors_initialized = 0;
		}
		break;
	}

	case id_checkbox_ToggleRightTrackLeftMode: {
		right_track_left_mode = evt.IsChecked();
		if (right_track_left_mode)
			initialize_transforms(mWorld);
		break;
	}

	default: {}
	}
}

/* ********************************************************************************************* */
void SimTab::OnSlider(wxCommandEvent &evt) {}

/* ********************************************************************************************* */
// Handler for events

BEGIN_EVENT_TABLE(SimTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimTab::OnSlider)
//EVT_CHECKBOX(wxEVT_COMMAND_CHECKBOX_CLICKED, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleMotorInputMode, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleMotorOutputMode, SimTab::OnButton)
EVT_CHECKBOX(id_checkbox_ToggleRightTrackLeftMode, SimTab::OnButton)
END_EVENT_TABLE()

/* ********************************************************************************************* */
// Class constructor for the tab: Each tab will be a subclass of GRIPTab

IMPLEMENT_DYNAMIC_CLASS(SimTab, GRIPTab)

/* ********************************************************************************************* */
// Necessary interface call to create a GRIP executable 

/// Application class
extern wxNotebook* tabView;
class mainApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new SimTab(tabView), wxT("Teleop"));
	}
};

IMPLEMENT_APP(mainApp)



/*
 * SANDBOX FOR HACKING
 */

//filter_kalman_t *kf;					///< the kalman filter to smooth the imu readings
//
///// Computes the imu value from the imu readings
//void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt,
//		filter_kalman_t* kf) {
//
//	// ======================================================================
//	// Get the readings
//
//	// Get the value
//	int r;
//	struct timespec currTime;
//	clock_gettime(CLOCK_MONOTONIC, &currTime);
//	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
//	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector,
//			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, imuChan, &abstime );
//	assert((imu_msg != NULL) && "Imu message is faulty!");
//
//	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
//	static const double mountAngle = -.7853981634;
//	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
//	_imu = atan2(newX, imu_msg->data[2]);
//	_imuSpeed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);
//
//	// Free the unpacked message
//	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );
//
//	// ======================================================================
//	// Filter the readings
//
//	// Skip if a filter is not provided
//	if(kf == NULL) return;
//
//	// Setup the data
//	kf->z[0] = _imu, kf->z[1] = _imuSpeed;
//
//	// Setup the time-dependent process matrix
//	kf->A[0] = kf->A[3] = 1.0;
//	kf->A[2] = dt;
//
//	// Setup the process noise matrix
//	static const double k1 = 2.0;
//	static const double k1b = 5.0;
//	kf->R[0] = (dt*dt*dt*dt) * k1 * (1.0 / 4.0);
//	kf->R[1] = (dt*dt*dt) * k1 * (1.0 / 2.0);
//	kf->R[2] = (dt*dt*dt) * k1 * (1.0 / 2.0);
//	kf->R[3] = (dt*dt) * k1b;
//
//	// First make a prediction of what the reading should have been, then correct it
//	filter_kalman_predict(kf);
//	filter_kalman_correct(kf);
//
//	// Set the values
//	_imu = kf->x[0], _imuSpeed = kf->x[1];
//}
//
//void initIMUFilter() {
//
//	// Set the offset values to amc motor group so initial wheel pos readings are zero
//	double imu, imuSpeed;
//	getImu(&imuChan, imu, imuSpeed, 0.0, NULL);
//
//	usleep(1e5);
//
//	// Initialize kalman filter for the imu and set the measurement and meas. noise matrices
//	// Also, set the initial reading to the current imu reading to stop moving from 0 to current
//	kf = new filter_kalman_t;
//	filter_kalman_init(kf, 2, 0, 2);
//	kf->C[0] = kf->C[3] = 1.0;
//	kf->Q[0] = kf->Q[3] = 1e-3;
//	kf->x[0] = imu, kf->x[1] = imuSpeed;
//}

/*
 * ENDSANDBOX
 */
