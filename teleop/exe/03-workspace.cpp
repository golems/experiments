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
VectorXd transformToEuler(const MatrixXd &T) {
	// extract translation
	Vector3d posV = T.topRightCorner<3,1>();

	// convert rotmat to euler
	Matrix3d rotM = T.topLeftCorner<3,3>();
	Vector3d rotV = math::matrixToEuler(rotM, math::XYZ);  // math::ZYX for spacenav!?

	// pack into a 6D config vector
	VectorXd V(6);
	V << posV, rotV;
	return V;
}

MatrixXd eulerToTransform(const VectorXd &V) {
	// extract translation
	Vector3d posV; posV << V[0], V[1], V[2];

	// extract rotation
	Vector3d rotV; rotV << V[3], V[4], V[5];

	// convert rotmat to euler
	Matrix3d rotM = math::eulerToMatrix(rotV, math::XYZ);  // math::ZYX for spacenav!?

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

/*
 * Obtains arm configurations from dart world
 */
void getArmConfigs(const simulation::World* world, VectorXd &qL, VectorXd &qR) {
	qL = world->getSkeleton("Krang")->getConfig(armIDsL);
	qR = world->getSkeleton("Krang")->getConfig(armIDsR);
}

/*
 * Obtains arm configs from somatic channel
 */
void getArmConfigs(VectorXd &qL, VectorXd &qR) {
//	somatic_motor_update(&daemon_cx, &llwa);
//	for(size_t i = 0; i < 7; i++) vals(i) = llwa.pos[i];
//	vector <int> arm_ids;
//	for(size_t i = 4; i < 17; i+=2) arm_ids.push_back(i + 6);
//	mWorld->getSkeleton(krang_id)->setConfig(arm_ids, vals);
}

/*
 *sets arm configs in the dart world
 */
void setArmConfigs(const simulation::World* world, VectorXd &qL, VectorXd &qR) {
	world->getSkeleton("Krang")->setConfig(armIDsL, qL);
	world->getSkeleton("Krang")->setConfig(armIDsR, qR);
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

	//TODO initialize pose configs?
	qL = VectorXd(7);
	qR = VectorXd(7);
}

/* ********************************************************************************************* */
/*
 * use this one when you want to interpret ui input as poses
 *
 * Computes new goal pose for effectors according to the current liberty poses
 * That is, given the displacement of the current sensor reading from the initial, computes
 * where the reference end-effector pose should be, using the initial one.
 */
void computeEffectorGoalPoses(const MatrixXd &eeL_init, const MatrixXd &eeR_init,
		const MatrixXd &uiL_init, const MatrixXd &uiR_init,
		const MatrixXd &uiL_cur, const MatrixXd &uiR_cur,
		MatrixXd &eeL_goal, MatrixXd &eeR_goal) {

	// compute liberty transforms from initial pose
	cout << "\nuiL_init:\n" << uiL_init << endl;
	cout << "\nuiL_cur:\n" << uiL_cur << endl;
	MatrixXd uiL_delta = uiL_init.inverse() * uiL_cur;
	MatrixXd uiR_delta = uiR_init.inverse() * uiR_cur;
	cout << "\nuiL_delta:\n" << uiL_delta << endl;

//	MatrixXd transformFix = Matrix4d::Zero();
//	transformFix.topLeftCorner<3,3>() = eeL_init.topLeftCorner<3,3>().transpose() * uiL_init.topLeftCorner<3,3>();
//	transformFix(3,3) = 1.0;
//	cout << "\ttransformFix:\n" << transformFix << endl;

	// for now, just use those as goal
	cout << "\neeL_init:\n" << eeL_init << endl;
	eeL_goal = uiL_cur;//eeL_init * uiL_delta;
	cout << "\neeL_goal:\n" << eeL_goal << endl;
	eeR_goal = eeR_init * uiR_delta;
}

/*
 * use this one when you want to interpret UI input as velocities
 */
void updateEffectorGoalFromUI(MatrixXd &goalM, MatrixXd &ui_curM, VectorXd &goalE, double gain = 0.003) {
	goalE = transformToEuler(goalM);
	VectorXd ui_curE(6);
	ui_curE = transformToEuler(ui_curM);
	goalE += (ui_curE * gain); // update goal in Euler space
	goalM = eulerToTransform(goalE); // update matrix representation
}

/* ********************************************************************************************* */
/// Given the two goal 4x4 transformation for the arrows, draw them
void setGoalSkelConfigs(const MatrixXd &goalL, const MatrixXd &goalR) {
	mWorld->getSkeleton("g1")->getNode("link_0")->setWorldTransform(goalL);
	mWorld->getSkeleton("g2")->getNode("link_0")->setWorldTransform(goalR);
//	const MatrixXd *goal_transforms[] = {&goalL, &goalR};
//	dynamics::SkeletonDynamics* skeletons[] = {mWorld->getSkeleton("g1"), mWorld->getSkeleton("g2")};
//
//	for (int i=0; i < 2; i++ ) {
//		// grab ref to goal transform
//		const MatrixXd *goal = goal_transforms[i];
//		// grab ref to effector skeleton
//		SkeletonDynamics *skel = skeletons[i];
//
//		// convert goal to euler
//		VectorXd arrowConf = transformToEuler(*goal);
//
//		// set config of skeleton
//		vector<int> conf_ids;
//		for(int k = 0; k < 6; k++) conf_ids.push_back(k);
//		skel->setConfig(conf_ids, arrowConf);
//	}
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

/* ********************************************************************************************* */
/* @DEPRECATED
 * Computes desired workspace effector velocities xdot based on the
 * provided goals, and returns jointspace velocities qdot from jacobian
 */
void computeJointVelocities(simulation::World* world,
		MatrixXd &eeL_cur, MatrixXd &eeR_cur,
		MatrixXd &eeL_goal, MatrixXd &eeR_goal,
		VectorXd &qdotl, VectorXd &qdotr) {

	// Bundle up all our arguments so we can do this in a loop
	MatrixXd *cur_transforms[] = {&eeL_cur, &eeR_cur};
	MatrixXd *goal_transforms[] = {&eeL_goal, &eeR_goal};
	kinematics::BodyNode* ee_nodes[] = {world->getSkeleton("Krang")->getNode("lGripper"),
			world->getSkeleton("Krang")->getNode("rGripper")};
	VectorXd *qdots[] = {&qdotl, &qdotr};

	for (int i=0; i < 2; i++) {
		// compute error transform
//		MatrixXd errM = cur_transforms[i]->inverse() * (*goal_transforms[i]);

		// convert to 6D euler
//		VectorXd xdot = transform_to_euler(errM);

		VectorXd xcur = transformToEuler(*cur_transforms[i]);
		VectorXd xgoal = transformToEuler(*goal_transforms[i]);
		VectorXd xdot(6);
		xdot = xgoal - xcur;

		// scale dimensions before we start
		double weights[] = {0.5, 0.5, 0.5, 0.1, 0.1, 0.1};
		for (int j=0; j < 3; j++) xdot[j] *= weights[j];

		cout << "left cur: " << xcur.transpose() << endl;
		cout << "left goal: " << xgoal.transpose() << endl;


		if (i==0)
			cout << "xdot: " << i << " " << xdot.transpose() << endl;

		// map to joint-space through jacobian
		(*qdots[i]) = workToJointVelocity(ee_nodes[i], xdot);
	}
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
	static const double kSpeed = 0.01;
	xdot << errPos, errOriV;
	xdot = kSpeed * xdot.normalized();
	return xdot;
}

/*
 * Packs the provided motor velocities into a somatic motor message
 * sends along its merry way
 */
void dispatchMotorVels() {

}

/* ********************************************************************************************* */
/*
 * Manually updates the arm configuration for the provided jointspace
 * velocities, scaled by the provided dt
 * TODO use dart dynamics
 */
void fakeArmMovement(VectorXd &qL, VectorXd &qR, VectorXd &qdotL, VectorXd &qdotR, double dt=0.015) {
	getArmConfigs(mWorld, qL, qR);
	qL += (qdotL * dt);
	qR += (qdotR * dt);
	setArmConfigs(mWorld, qL, qR);
	cout << "qdot L: " << qdotL.transpose() << endl;
	cout << "qL: " << qL.transpose() << endl;
}

/* ********************************************************************************************* */
/*
 * Picks a random configuration for the robot, moves it, does f.k. for the right and left
 * end-effectors, places blue and green boxes for their locations and visualizes it all
 */
void Timer::Notify() {
	// get current poses
	//getEffectorPoses(mWorld, T_eeL_cur, T_eeR_cur);

	// pack ui input
//	MatrixXd *uiL_init, *uiR_init, *uiL_cur, *uiR_cur;
//	if (input_mode == UI_LIBERTY) {
//		MatrixXd *Tlibs[] = {&T_lib1_cur, &T_lib2_cur};
//		getLibertyPoses(Tlibs, 2, NULL);
//		uiL_init = &T_lib1_init;
//		uiR_init = &T_lib2_init;
//		uiL_cur = Tlibs[0];
//		uiR_cur = Tlibs[1];
//
//		// use libertys as pose goals
//		computeEffectorGoalPoses(T_eeL_init, T_eeR_init, *uiL_init, *uiR_init,
//				*uiL_cur, *uiR_cur, T_eeL_goal, T_eeR_goal);
//	} else {
//		getJoystickPose(T_joy_cur);
//		uiL_init = &T_joy_init;
//		uiR_init = &T_dummy;
//		uiL_cur = &T_joy_cur;
//		uiR_cur = &T_dummy;
//
//		// use joystick as velocity goal
//		VectorXd eeL_goal(6), eeR_goal(6);
//		updateEffectorGoalFromUI(T_eeL_goal, *uiL_cur, eeL_goal, 0.1);
//		updateEffectorGoalFromUI(T_eeR_goal, *uiR_cur, eeR_goal, 0.1);
//	}

	// set goal arrow configurations
	//setGoalSkelConfigs(T_eeL_goal, T_eeR_goal);
	T_eeL_goal = mWorld->getSkeleton("g1")->getNode("link_0")->getWorldTransform();
	T_eeR_goal = mWorld->getSkeleton("g2")->getNode("link_0")->getWorldTransform();

	// compute goal errors, gain and pump through arm jacobians
	//computeJointVelocities(mWorld, T_eeL_cur, T_eeR_cur, T_eeL_goal, T_eeR_goal, qdotL, qdotR);

	// grab effector node references
	kinematics::BodyNode *eeL_node = mWorld->getSkeleton("Krang")->getNode("lGripper");
	kinematics::BodyNode *eeR_node = mWorld->getSkeleton("Krang")->getNode("rGripper");

	// compute workspace velocity from goal errors
	VectorXd xdotL = computeWorkVelocity(eeL_node, T_eeL_goal);
	VectorXd xdotR = computeWorkVelocity(eeR_node, T_eeR_goal);

	// convert to jointspace
	VectorXd qdotL = workToJointVelocity(eeL_node, xdotL);
	VectorXd qdotR = workToJointVelocity(eeR_node, xdotR);

	// handle arm velocities
	if (motor_mode)
		dispatchMotorVels();
	else
		fakeArmMovement(qL, qR, qdotL, qdotR);

	// Restart the timer for the next start
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

	// Initialize the ach channel for liberty
	initLiberty();
	initJoystick();

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// set arm indices and configs
	initialize_configs(mWorld);

	// Manually set the initial arm configuration for the left arm
	VectorXd larm_conf(7), rarm_conf(7);
	larm_conf << 0.0, -M_PI / 3.0, 0.0, -M_PI / 3.0, 0.0, M_PI/6.0, 0.0;
	rarm_conf << 0.0, M_PI / 3.0, 0.0, M_PI / 3.0, 0.0, M_PI/6.0, 0.0;
	mWorld->getSkeleton("Krang")->setConfig(armIDsL, larm_conf);
	mWorld->getSkeleton("Krang")->setConfig(armIDsR, rarm_conf);

	// Initialize primary transforms
	usleep(1e5); // give ach time to initialize, or init transforms will be bogus
	initialize_transforms(mWorld);

	getEffectorPoses(mWorld, T_eeL_cur, T_eeR_cur);
	setGoalSkelConfigs(T_eeL_cur, T_eeR_cur);
}

/* ********************************************************************************************* */
SimTab::~SimTab() {
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
