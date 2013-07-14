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
#include "liberty_client.h"
#include <math/UtilsRotation.h>

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
	id_button_ResetScene= 8100,
	id_button_ResetLiberty= 8101,
	id_button_ResetArms= 8102,
};

// control globals
static bool motor_mode = 0;

// Effector and liberty transforms
MatrixXd T_eel_init; //< left effector global transform /
MatrixXd T_eer_init; //< right effector global transform /
MatrixXd T_lib1_init; //< liberty channel1 global transform /
MatrixXd T_lib2_init; //< liberty channel2 global transform /

MatrixXd T_eeL_cur; //< left effector global transform /
MatrixXd T_eeR_cur; //< right effector global transform /
MatrixXd T_lib1_cur; //< liberty channel1 global transform /
MatrixXd T_lib2_cur; //< liberty channel2 global transform /

MatrixXd T_eeL_goal; //< liberty channel1 global transform /
MatrixXd T_eeR_goal; //< liberty channel2 global transform /

// Arm configurations
VectorXd qL;
VectorXd qR;
VectorXd qdotL;
VectorXd qdotR;
vector<int> armIDsL;
vector<int> armIDsR;

/************************ Helpers **************************/

/*
 * Converts a 4x4 homogeneous transform to a 6D euler
 */
VectorXd transform_to_euler(const MatrixXd &T) {
	// extract translation
	Vector3d posV = T.topRightCorner<3,1>();

	// convert rotmat to euler
	Matrix3d rotM = T.topLeftCorner<3,3>();
	Vector3d rotV = math::matrixToEuler(rotM, math::XYZ);

	// pack into a 6D config vector
	VectorXd V(6);
	V << posV, rotV;
	return V;
}

/*
 * Queries dart krang skeleton for current effector poses
 */
bool getEffectorPoses(simulation::World* world, MatrixXd &ee_left, MatrixXd &ee_right) {
	kinematics::BodyNode* eelNode = world->getSkeleton("Krang")->getNode("lgPlate1");
	ee_left = eelNode->getWorldTransform();
	kinematics::BodyNode* eerNode = world->getSkeleton("Krang")->getNode("rgPlate1");
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
	T_eel_init= Matrix4d(4,4); //< left effector global transform
	T_eer_init = Matrix4d(4,4); //< right effector global transform
	T_lib1_init = Matrix4d(4,4); //< liberty channel1 global transform
	T_lib2_init = Matrix4d(4,4); //< liberty channel2 global transform

	T_eel_init= Matrix4d(4,4); //< left effector global transform
	T_eeR_cur = Matrix4d(4,4); //< right effector global transform
	T_lib1_cur = Matrix4d(4,4); //< liberty channel1 global transform
	T_lib2_cur = Matrix4d(4,4); //< liberty channel2 global transform

	T_eeL_goal = Matrix4d(4,4); //< left effector goal transform
	T_eeR_goal = Matrix4d(4,4); //< left effector global transform

	// grab pose of robot left and right effectors
	getEffectorPoses(world, T_eel_init, T_eer_init);

	// grab pose of liberty channels 1 and 2
	MatrixXd *Tlibs[] = {&T_lib1_init, &T_lib2_init};
	getLibertyPoses(Tlibs, 2, NULL);

	// set the effector goal poses to their current states
	T_eeL_goal = T_eel_init;
	T_eeR_goal = T_eer_init;
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
	qdotL = VectorXd(7);
	qdotR = VectorXd(7);
}

/*
 * Computes new goal pose for effectors according to the current liberty poses
 */
void computeEffectorGoalPoses(const MatrixXd &eel_init, const MatrixXd &eer_init,
		const MatrixXd &lib1_init, const MatrixXd &lib2_init,
		const MatrixXd &lib1_cur, const MatrixXd &lib2_cur,
		MatrixXd &eel_goal, MatrixXd &eer_goal) {

	// compute liberty transforms from initial pose
	MatrixXd lib1_delta = lib1_init.inverse() * lib1_cur;
	MatrixXd lib2_delta = lib2_init.inverse() * lib2_cur;

	// for now, just use those as goal
	eel_goal = eel_init * lib1_delta;
	eer_goal = eer_init * lib2_delta;
}

/*
 *
 */
void setGoalSkelConfigs(MatrixXd &T_eel_goal, MatrixXd &T_eer_goal) {
	MatrixXd *goal_transforms[] = {&T_eel_goal, &T_eer_goal};
	dynamics::SkeletonDynamics* skeletons[] = {mWorld->getSkeleton("g1"), mWorld->getSkeleton("g2")};

	for (int i=0; i < 2; i++ ) {
		MatrixXd *Tgoal = goal_transforms[i];
		SkeletonDynamics *skel = skeletons[i];

		VectorXd arrowConf = transform_to_euler(*Tgoal);

		vector <int> conf_ids;
		for(size_t k = 0; k < 6; k++) conf_ids.push_back(k);
		skel->setConfig(conf_ids, arrowConf);
	}
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
	for (int i=0; i < Jsq.rows(); i++)
		Jsq.diagonal()[i] += 0.0001;
	Eigen::MatrixXd Jinv = Jt * Jsq.inverse();

	// Get the joint-space velocities by multiplying inverse Jacobian with x.
	VectorXd qdot = Jinv * xdot;
	return qdot;
}

/*
 *
 */
void computeJointVelocities(simulation::World* world, MatrixXd &eel_cur,
		MatrixXd &eer_cur, MatrixXd &eel_goal, MatrixXd &eer_goal,
		VectorXd &qdotl, VectorXd &qdotr) {

	// Bundle up all our arguments so we can do this in a loop
	MatrixXd *cur_transforms[] = {&eel_cur, &eer_cur};
	MatrixXd *goal_transforms[] = {&eel_goal, &eer_goal};
	kinematics::BodyNode* ee_nodes[] = {world->getSkeleton("Krang")->getNode("lgPlate1"),
			world->getSkeleton("Krang")->getNode("rgPlate1")};
	VectorXd *qdots[] = {&qdotl, &qdotr};

	for (int i=0; i < 2; i++) {
		// compute error transform
		MatrixXd errM = cur_transforms[i]->inverse() * (*goal_transforms[i]);

		// convert to 6D euler
		VectorXd xdot = transform_to_euler(errM);
		xdot *= 0.1;

		// map to joint-space through jacobian
		(*qdots[i]) = workToJointVelocity(ee_nodes[i], xdot);
	}
}

/*
 * Packs the provided motor velocities into a somatic motor message
 * sends along its merry way
 */
void dispatchMotorVels() {

}

/*
 * Manually updates the arm configuration for the provided jointspace
 * velocities, scaled by the provided dt
 */
void fakeArmMovement(VectorXd &qL, VectorXd &qR, VectorXd &qdotL, VectorXd &qdotR, double dt=0.03) {
	getArmConfigs(mWorld, qL, qR);
	qL += qdotL * dt;
	qR += qdotR * dt;
	setArmConfigs(mWorld, qL, qR);
}

/* ********************************************************************************************* */
/*
 * Picks a random configuration for the robot, moves it, does f.k. for the right and left
 * end-effectors, places blue and green boxes for their locations and visualizes it all
 */
void Timer::Notify() {
	// get current poses
	getEffectorPoses(mWorld, T_eeL_cur, T_eeR_cur);
	MatrixXd *Tlibs[] = {&T_lib1_cur, &T_lib2_cur};
	getLibertyPoses(Tlibs, 2, NULL);

	// compute new goal pose
	computeEffectorGoalPoses(T_eel_init, T_eer_init, T_lib1_init, T_lib2_init,
			T_lib1_cur, T_lib2_cur, T_eeL_goal, T_eeR_goal);

	// set goal arrow configurations
	setGoalSkelConfigs(T_eeL_goal, T_eeR_goal);

	// compute goal errors, gain and pump through arm jacobians
	computeJointVelocities(mWorld, T_eeL_cur, T_eeR_cur, T_eeL_goal, T_eeR_goal, qdotL, qdotR);

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
	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Initialization"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Control"));
	wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Random"));

	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
	wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

	ss1BoxS->Add(new wxButton(this, id_button_ResetScene, wxT("Reset Scene")), 0, wxALL, 1);
	ss1BoxS->Add(new wxButton(this, id_button_ResetLiberty, wxT("Set Liberty Initial Transforms")), 0, wxALL, 1);
	ss1BoxS->Add(new wxButton(this, id_button_ResetArms, wxT("Set Arm Initial Transforms")), 0, wxALL, 1);

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
	ach_init(&liberty_chan, "liberty", achbuf_liberty, n_achbuf_liberty);

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Initialize primary transforms
	usleep(1e4); // give ach time to initialize, or init transforms will be bogus
	initialize_transforms(mWorld);
	initialize_configs(mWorld);
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
		break;
	}

	case id_button_ResetLiberty: {
		MatrixXd *Tlibs[] = {&T_lib1_init, &T_lib2_init};
		getLibertyPoses(Tlibs, 2, NULL);
		break;
	}

	case id_button_ResetArms: {
		getEffectorPoses(mWorld, T_eel_init, T_eer_init);
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
