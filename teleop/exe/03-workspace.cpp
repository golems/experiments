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
#include "somatic/daemon.h"
#include <somatic/motor.h>

#include <math/UtilsRotation.h>

#include "util.h"
#include "initModules.h"
#include "liberty_client.h"
#include "joystick_client.h"


using namespace std;
using namespace Eigen;
using namespace dynamics;

/* ********************************************************************************************* */

// somatic globals
somatic_d_t daemon_cx;
somatic_motor_t llwa;
somatic_motor_t rlwa;

// UI Globals
/* Events */
enum DynamicSimulationTabEvents {
	id_button_ResetScene = 8100,
	id_button_ResetLiberty,
	id_button_ResetJoystick,
	id_button_ResetArms,
	id_checkbox_ToggleMotorInputMode,
	id_checkbox_ToggleMotorOutputMode,
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
static int ui_input_mode = UI_JOYSTICK;

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

MatrixXd T_joy_init;	//< joystick device initial transform
MatrixXd T_joy_cur;		//< joystick device current transform

MatrixXd T_dummy = MatrixXd(4,4).setIdentity(4,4); // dummy transform for 1-arm control

// Arm configurations
VectorXd qL;
VectorXd qR;
vector<int> armIDsL;
vector<int> armIDsR;


/*
 * Queries dart krang skeleton for current effector poses
 */
bool getEffectorPoses(simulation::World* world, MatrixXd &ee_left, MatrixXd &ee_right) {
	kinematics::BodyNode* eelNode = world->getSkeleton("Krang")->getNode("lGripper");
	ee_left = eelNode->getWorldTransform();
	kinematics::BodyNode* eerNode = world->getSkeleton("Krang")->getNode("rGripper");
	ee_right = eerNode->getWorldTransform();
}

/*
 * Updates arm configurations in krang skeleton from current
 * somatic motor states
 */
void updateRobotSkelFromSomatic(simulation::World* world, somatic_d_t &daemon_cx,
		somatic_motor_t &arm, vector<int> &armIDs) {

	VectorXd vals(7);
	somatic_motor_update(&daemon_cx, &arm);
	for(size_t i = 0; i < 7; i++)
		vals(i) = arm.pos[i];
	world->getSkeleton("Krang")->setConfig(armIDs, vals);
}

/*
 * Send joint velocities to robot over somatic
 */
void sendRobotArmVelocities(somatic_d_t &daemon_cx, somatic_motor_t &arm, VectorXd &qdot,
		double gain = 0.02) {

	qdot = qdot.normalized() * gain;
	double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for(size_t i = 0; i < 7; i++)
		dq[i] = qdot(i);
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
}

/************************ Initialization **************************/
/*
 * Caches all relevant transforms from the robot's current state
 */
void initialize_transforms(simulation::World* world) {
	T_joy_init = Matrix4d(4,4); //< initial joystick transform
	T_joy_cur = Matrix4d(4,4); //< current joystick transform

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

	// grab pose of robot left and right effectors
	getEffectorPoses(world, T_eeL_init, T_eeR_init);

	// grab pose of liberty channels 1 and 2
	MatrixXd *Tlibs[] = {&T_lib1_init, &T_lib2_init};
	getLibertyPoses(Tlibs, 2, NULL);

	// grab joystick pose
	getJoystickPose(T_joy_init);

	// set the effector goal poses to their current states
	T_eeL_goal = T_eeL_init;
	T_eeR_goal = T_eeR_init;
}

/*
 * Initialize arm configuration IDs
 */
void initialize_configs(simulation::World* world) {
	static const int idL[] = {11, 13, 15, 17, 19, 21, 23};
	static const int idR[] = {12, 14, 16, 18, 20, 22, 24};
	armIDsL = vector<int>(idL, idL + sizeof(idL)/sizeof(idL[0]));
	armIDsR = vector<int>(idR, idR + sizeof(idR)/sizeof(idR[0]));
}

/* ********************************************************************************************* */
/// Given the two goal 4x4 transformation for the arrows, draw them
void setGoalSkelConfig(dynamics::SkeletonDynamics *goalSkel, const MatrixXd &goal) {
	// convert goal to euler
	VectorXd arrowConf = transformToEuler(goal, dartRotOrder);

	// set config of skeleton
	vector<int> conf_ids;
	for(int k = 0; k < 6; k++) conf_ids.push_back(k);
	goalSkel->setConfig(conf_ids, arrowConf);
}

/****************************************************************************************
 * Workspace goal update functions
 ****************************************************************************************/
/// Returns the goal configuration for the end-effector using spacenav
void updateGoalFromSpnavVels(kinematics::BodyNode *eeNode, MatrixXd& goalTransform,
		dynamics::SkeletonDynamics *goalSkel = NULL, double scale = 0.1) {

	/*
	 * TODO:
	 * 1) un-reverse spacenav pitch dimension
	 * 2) get jacobian for pitch dimension to work with eulerToTransform
	 * 3) drift thing
	 */

	// Get the config of the spacenav
	Vector6d goalConfig;
	getJoystickPose(T_eeL_goal, &goalConfig);

	// scale workspace velocity dims
	double weights[] = {1, 1, 1, 2, 2, 2};
	for (int i=0; i < 6; ++i)
		goalConfig[i] *= weights[i];

	// Set the location around the current end-effector pose with some scale
	Matrix4d eeT = eeNode->getWorldTransform();
	goalConfig.topLeftCorner<3,1>() = eeT.topRightCorner<3,1>() + scale * goalConfig.topLeftCorner<3,1>();

	// Set the orientation "change" with respect to the end-effector orientation
	Quaternion<double> goalOri(T_eeL_goal.topLeftCorner<3,3>());
	Quaternion <double> eeOri(eeT.topLeftCorner<3,3>());
	Quaternion<double> errOriQ = goalOri * eeOri.inverse();
	Matrix3d errOriM = errOriQ.matrix();
	goalConfig.bottomLeftCorner<3,1>() = math::matrixToEuler(errOriM, dartRotOrder);


	// update goalTransform from config representation
	goalTransform = eulerToTransform(goalConfig, dartRotOrder);
//	cout << "goal transform from euler conversion\n" << goalTransform << endl;

	// Update the dart data structure and set the goal
	if (goalSkel != NULL) {
		vector <int> obj_idx;
		for(int i = 0; i < 6; i++) obj_idx.push_back(i);
		goalSkel->setConfig(obj_idx, goalConfig);
		goalTransform = goalSkel->getNode("link_0")->getWorldTransform(); //TODO deleteme!

//		cout << "goal transform from dart\n" << goalTransform << endl <<endl;
	}
}

/****************************************************************************************
 * Jacobian velocity functions
 ****************************************************************************************/
/*
 * Computes desired workspace effector velocities xdot based on the
 * provided goals, and returns jointspace velocities qdot from jacobian
 */
VectorXd computeWorkVelocity(const kinematics::BodyNode* eeNode, const MatrixXd &eeGoal ) {

	VectorXd xdot(6); xdot.Zero(6);

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
	Vector3d errOriV = math::matrixToEuler(errOriM, dartRotOrder);

	// Check if the goal is reached
	static const double posLimit = 0.01;
	static const double oriLimit = 0.01;
	if((errPos.norm() < posLimit) && (errOriV.norm() < oriLimit))
		return xdot;

	// Get the workspace velocity
	//errOriV= Vector3d(0.0, 0.0, 0.0);
	xdot << errPos, errOriV;
	return xdot;
}

/*
 * Given a workspace velocity, returns the joint space velocity
 */
VectorXd workToJointVelocity (kinematics::BodyNode* eeNode, const VectorXd& xdot) {

	// Get the Jacobian towards computing joint-space velocities
	MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jsq = J * Jt;
//	for (int i=0; i < Jsq.rows(); i++)
//		Jsq.diagonal()[i] += 0.0001;
	Eigen::MatrixXd Jinv = Jt * Jsq.inverse();

	// Get the joint-space velocities by multiplying inverse Jacobian with x.
	VectorXd qdot = Jinv * xdot;
	return qdot;
}

/* ********************************************************************************************* */
/*
 * Manually updates the arm configuration for the provided jointspace
 * velocities, scaled by the provided dt
 * TODO use dart dynamics
 */
void fakeArmMovement(VectorXd &qL, VectorXd &qR, VectorXd &qdotL, VectorXd &qdotR, double dt=0.035) {
	qL = mWorld->getSkeleton("Krang")->getConfig(armIDsL);
	qL += (qdotL.normalized() * dt);
	mWorld->getSkeleton("Krang")->setConfig(armIDsL, qL);
}

/* ********************************************************************************************* */


/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {
//	Vector6d joycfg;
//	getJoystickPose(T_joy_cur, &joycfg);
//	cout << " joy cfg: " << joycfg.transpose() << endl;
//	//cout << " joy transform: \n" << T_joy_cur << endl;
//	setGoalSkelConfig(mWorld->getSkeleton("g1"), T_joy_cur);

	// update robot state from ach if we're controlling the actual robot
	if (motor_input_mode) {
		updateRobotSkelFromSomatic(mWorld, daemon_cx, llwa, armIDsL);
		//updateRobotSkelFromSomatic(mWorld, daemon_cx, rlwa, armIDsR);
	}

	// Grab effector node pointer to compute the task space error and the Jacobian
	kinematics::BodyNode *eeL_node = mWorld->getSkeleton("Krang")->getNode("lGripper");

	// Get the goal configuration from spacenav
	updateGoalFromSpnavVels(eeL_node, T_eeL_goal, mWorld->getSkeleton("g1"), 0.1);

	// Compute workspace velocity from goal errors
	VectorXd xdotL = computeWorkVelocity(eeL_node, T_eeL_goal);

	// Convert to jointspace velocities
	VectorXd qdotL = workToJointVelocity(eeL_node, xdotL);

	// dispatch joint velocities to arms
	if (motor_output_mode) {
		sendRobotArmVelocities(daemon_cx, llwa, qdotL);
		//sendRobotArmVelocities(daemon_cx, rlwa, qdotR);
	} else {
		// Move the arms with a small delta using the computed velocities
		if(xdotL.norm() > 1e-2)
			fakeArmMovement(qL, qR, qdotL, qdotL);
	}

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
	ss2BoxS->Add(new wxButton(this, id_button_ResetArms, wxT("Set Arm Initial Transforms")), 0, wxALL, 1);

	ss3BoxS->Add(new wxCheckBox(this, id_checkbox_ToggleMotorOutputMode, wxT("Send Motor Commands")), 0, wxALL, 1);

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
	initialize_configs(mWorld);

	// Manually set the initial arm configuration for the left arm
	VectorXd larm_conf(7), rarm_conf(7);
	larm_conf << 0.0, -M_PI / 3.0, 0.0, -M_PI / 3.0, 0.0, M_PI/6.0, 0.0;
	mWorld->getSkeleton("Krang")->setConfig(armIDsL, larm_conf);

	// Initialize the joystick
	initJoystick();

	// Initialize the liberty
	initLiberty();

	// Initialize the arms
	if (motor_input_mode || motor_output_mode) {
		initArm(daemon_cx, llwa, "llwa");
		//initArm(daemon_cx, rlwa, "rlwa");
		motors_initialized = 1;
	}

	// Initialize primary transforms
	usleep(1e5); // give ach time to initialize, or init transforms will be bogus
	initialize_transforms(mWorld);
}

/* ********************************************************************************************* */
SimTab::~SimTab() {

	// halt the arms TODO: why isn't this being called?
//	if (motor_mode) {
//		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
//		//somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
//		cout << "halted motors" << endl;
//	}

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
		getJoystickPose(T_joy_init);
		break;
		}

	case id_button_ResetArms: {
		getEffectorPoses(mWorld, T_eeL_init, T_eeR_init);
		break;
	}

	case id_checkbox_ToggleMotorInputMode: {
		motor_input_mode = evt.IsChecked();
		if (motor_input_mode && !motors_initialized) {
			initArm(daemon_cx, llwa, "llwa");
			//initArm(daemon_cx, rlwa, "rlwa");
			motors_initialized = 1;
		}
		break;
	}

	case id_checkbox_ToggleMotorOutputMode: {

		motor_output_mode = evt.IsChecked();
		if (motor_output_mode && !motors_initialized) {
			initArm(daemon_cx, llwa, "llwa");
			//initArm(daemon_cx, rlwa, "rlwa");
			motors_initialized = 1;
		}

		if (!motor_output_mode) {
			cout << "halting motors" << endl;
			somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
			motors_initialized = 0;
		}


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
		tabView->AddPage(new SimTab(tabView), wxT("Liberty"));
	}
};

IMPLEMENT_APP(mainApp)
