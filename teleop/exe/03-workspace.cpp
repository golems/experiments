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
};

// control globals
enum UI_MODES {
	UI_LIBERTY = 0,
	UI_JOYSTICK,
	UI_MIXED,
};
static bool motor_mode = 0;
static int input_mode = UI_JOYSTICK;

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
//VectorXd qdotL;
//VectorXd qdotR;
vector<int> armIDsL;
vector<int> armIDsR;

/************************ Helpers **************************/

/*
 * Converts a 4x4 homogeneous transform to a 6D euler.
 * Conversion convention corresponds to Grip's ZYX
 */
VectorXd transformToEuler(const MatrixXd &T, math::RotationOrder _order = math::XYZ) { // math::XYZ
	// extract translation
	Vector3d posV = T.topRightCorner<3,1>();

	// convert rotmat to euler
	Matrix3d rotM = T.topLeftCorner<3,3>();
	Vector3d rotV = math::matrixToEuler(rotM, _order);  // math::ZYX for spacenav!?

	// pack into a 6D config vector
	VectorXd V(6);
	V << posV, rotV;
	return V;
}

MatrixXd eulerToTransform(const VectorXd &V, math::RotationOrder _order = math::XYZ) { // math::XYZ
	// extract translation
	Vector3d posV; posV << V[0], V[1], V[2];

	// extract rotation
	Vector3d rotV; rotV << V[3], V[4], V[5];

	// convert rotmat to euler
	Matrix3d rotM = math::eulerToMatrix(rotV, _order);  // math::ZYX for spacenav!?

	// pack into a 4x4 matrix
	MatrixXd T(4,4);
	T.topLeftCorner<3,3>() = rotM;
	T.topRightCorner<3,1>() = posV;
	T(3,3) = 1.0;

	return T;
}


/*
 * Queries dart krang skeleton for current effector poses
 */
bool getEffectorPoses(simulation::World* world, MatrixXd &ee_left, MatrixXd &ee_right) {
	kinematics::BodyNode* eelNode = world->getSkeleton("Krang")->getNode("lGripper");
	ee_left = eelNode->getWorldTransform();
	kinematics::BodyNode* eerNode = world->getSkeleton("Krang")->getNode("rGripper");
	ee_right = eerNode->getWorldTransform();
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
 *
 */
void initialize_configs(simulation::World* world) {
	static const int idL[] = {11, 13, 15, 17, 19, 21, 23};
	static const int idR[] = {12, 14, 16, 18, 20, 22, 24};
	armIDsL = vector<int>(idL, idL + sizeof(idL)/sizeof(idL[0]));
	armIDsR = vector<int>(idR, idR + sizeof(idR)/sizeof(idR[0]));
}

/*
 * use this one when you want to interpret UI input as velocities
 */
//void updateEffectorGoalFromUI(MatrixXd &goalM, MatrixXd &ui_curM, VectorXd &goalE, double gain = 0.003) {
//	goalE = transformToEuler(goalM);
//	VectorXd ui_curE(6);
//	ui_curE = transformToEuler(ui_curM);
//	goalE += (ui_curE * gain); // update goal in Euler space
//	goalM = eulerToTransform(goalE); // update matrix representation
//}

/* ********************************************************************************************* */
/// Given the two goal 4x4 transformation for the arrows, draw them
void setGoalSkelConfigs(const MatrixXd &goalL, const MatrixXd &goalR) {
//	mWorld->getSkeleton("g1")->getNode("link_0")->setWorldTransform(goalL);
//	mWorld->getSkeleton("g2")->getNode("link_0")->setWorldTransform(goalR);

	const MatrixXd *goal_transforms[] = {&goalL, &goalR};
	dynamics::SkeletonDynamics* skeletons[] = {mWorld->getSkeleton("g1"), mWorld->getSkeleton("g2")};

	for (int i=0; i < 2; i++ ) {
		// grab ref to goal transform
		const MatrixXd *goal = goal_transforms[i];
		// grab ref to effector skeleton
		SkeletonDynamics *skel = skeletons[i];

		// convert goal to euler
		VectorXd arrowConf = transformToEuler(*goal);

		// set config of skeleton
		vector<int> conf_ids;
		for(int k = 0; k < 6; k++) conf_ids.push_back(k);
		skel->setConfig(conf_ids, arrowConf);
	}
}

/* ********************************************************************************************* */
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

/*
 * Computes desired workspace effector velocities xdot based on the
 * provided goals, and returns jointspace velocities qdot from jacobian
 */
VectorXd computeWorkVelocity(kinematics::BodyNode* eeNode, MatrixXd &eeGoal ) {
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
	Vector3d errOriV = math::matrixToEuler(errOriM, math::XYZ);

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

/* ********************************************************************************************* */
/*
 * Manually updates the arm configuration for the provided jointspace
 * velocities, scaled by the provided dt
 * TODO use dart dynamics
 */
void fakeArmMovement(VectorXd &qL, VectorXd &qR, VectorXd &qdotL, VectorXd &qdotR, double dt=0.005) {
	qL = mWorld->getSkeleton("Krang")->getConfig(armIDsL);
	qL += (qdotL.normalized() * dt);
	cout << "qL: " << qL.transpose() << endl;
	mWorld->getSkeleton("Krang")->setConfig(armIDsL, qL);
}

/* ********************************************************************************************* */
/// Returns the goal configuration for the end-effector using spacenav
void updateGoalFromSpnavVels(kinematics::BodyNode *ee_node, MatrixXd& goal,
		dynamics::SkeletonDynamics *goal_skel = NULL, double scale = 0.1) {

	// Get the goal configuration from the spacenav
	Vector6d goalConfig;
	getJoystickPose(T_eeL_goal, &goalConfig);

	// Set the location around the current end-effector pose with some scale
	Matrix4d eeT = ee_node->getWorldTransform();
	goalConfig.topLeftCorner<3,1>() = eeT.topRightCorner<3,1>() + scale * goalConfig.topLeftCorner<3,1>();
	

	// Set the orientation "change" with respect to the end-effector orientation
	Quaternion<double> goalOri(T_eeL_goal.topLeftCorner<3,3>()); 
	Quaternion <double> eeOri(eeT.topLeftCorner<3,3>());
	Quaternion<double> errOriQ = goalOri * eeOri.inverse();
	Matrix3d errOriM = errOriQ.matrix();
	goalConfig.bottomLeftCorner<3,1>() = math::matrixToEuler(errOriM, math::XYZ);

	// update goal transform from config representation
	goal = eulerToTransform(goalConfig);

	// Update the dart data structure and set the goal
	if (goal_skel != NULL) {
		vector <int> obj_idx;
		for(int i = 0; i < 6; i++) obj_idx.push_back(i);
		goal_skel->setConfig(obj_idx, goalConfig);
		goal = goal_skel->getNode("link_0")->getWorldTransform();
		//cout << "goal from dart: " << goal << endl;
		//cout << "goal from conversion" << eulerToTransform(goalConfig) << endl;
	}
}

/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {

	// update robot state from ach if we're controlling the actual robot
	if (motor_mode) {

	}


	// Grab effector node pointer to compute the task space error and the Jacobian
	kinematics::BodyNode *eeL_node = mWorld->getSkeleton("Krang")->getNode("lGripper");
	cout <<"left pos: "<< eeL_node->getWorldTransform().topRightCorner<3,1>().transpose() << endl;

	// Get the goal configuration from spacenav
	updateGoalFromSpnavVels(eeL_node, T_eeL_goal, mWorld->getSkeleton("g1"), 0.1);
	
	// Set goal arrow configurations from the grip interface
//	T_eeL_goal = mWorld->getSkeleton("g1")->getNode("link_0")->getWorldTransform();
//	cout << "\ngoal pos: " << T_eeL_goal.topRightCorner<3,1>().transpose() << endl;


	// Compute workspace velocity from goal errors
	VectorXd xdotL = computeWorkVelocity(eeL_node, T_eeL_goal);
	cout << "xdot L: " << xdotL.transpose() << endl;
	cout << "xdot norm: " << xdotL.norm() << endl;

	// Convert to jointspace velocities
	VectorXd qdotL = workToJointVelocity(eeL_node, xdotL);
	cout << "qdot L: " << qdotL.transpose() << endl;

	// dispatch joint velocities to arms
	if (motor_mode) {
		// send to motors

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

	ss2BoxS->Add(new wxButton(this, id_button_ResetScene, wxT("Reset Scene")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetLiberty, wxT("Set Liberty Initial Transforms")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetJoystick, wxT("Set Joystick Initial Transform")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ResetArms, wxT("Set Arm Initial Transforms")), 0, wxALL, 1);

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
	T_eeL_goal = Matrix4d(4,4); //< left effector goal transform

	// Initialize the arms
	if (motor_mode) {
		initArm(daemon_cx, llwa, "llwa");
		initArm(daemon_cx, rlwa, "rlwa");
	}


	// Initialize primary transforms
	usleep(1e5); // give ach time to initialize, or init transforms will be bogus
	// initialize_transforms(mWorld);
	// getEffectorPoses(mWorld, T_eeL_cur, T_eeR_cur);
	// setGoalSkelConfigs(T_eeL_cur, T_eeR_cur);
}

/* ********************************************************************************************* */
SimTab::~SimTab() {

	// halt the arms
	if (motor_mode) {
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
		somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	}

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
void SimTab::OnButton(wxCommandEvent & evt) {
	int slnum = evt.GetId();
	switch(slnum) {
	// Set Start Arm Configuration
	case id_button_ResetScene: {
//		T_eeL_goal = T_eeL_cur;
//		T_eeL_goal(0,3) += 0.1;
//		computeJointVelocities(mWorld, T_eeL_cur, T_eeR_cur, T_eeL_goal, T_eeR_goal, qdotL, qdotR);
//		fakeArmMovement(qL, qR, qdotL, qdotR);
		break;
	}

	case id_button_ResetLiberty: {
//		MatrixXd *Tlibs[] = {&T_lib1_init, &T_lib2_init};
//		getLibertyPoses(Tlibs, 2, NULL);

//		T_eeL_goal = T_eeL_cur;
//		T_eeL_goal(0,3) -= 0.1;
//		computeJointVelocities(mWorld, T_eeL_cur, T_eeR_cur, T_eeL_goal, T_eeR_goal, qdotL, qdotR);
//		fakeArmMovement(qL, qR, qdotL, qdotR);

		break;
	}

	case id_button_ResetJoystick: {
//			getJoystickPose(T_joy_init);

//		T_eeL_goal = T_eeL_cur;
//		T_eeL_goal(1,3) += 0.1;
//		computeJointVelocities(mWorld, T_eeL_cur, T_eeR_cur, T_eeL_goal, T_eeR_goal, qdotL, qdotR);
//		fakeArmMovement(qL, qR, qdotL, qdotR);

		break;
		}

	case id_button_ResetArms: {
//		getEffectorPoses(mWorld, T_eeL_init, T_eeR_init);

//		T_eeL_goal = T_eeL_cur;
//		T_eeL_goal(1,3) -= 0.1;
//		computeJointVelocities(mWorld, T_eeL_cur, T_eeR_cur, T_eeL_goal, T_eeR_goal, qdotL, qdotR);
//		fakeArmMovement(qL, qR, qdotL, qdotR);

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
