/**
 * @file 02-gripSpaceNav.cpp
 * @author Can Erdogan, Saul Reynolds-Haertle, Jon Scholtz
 * @date July 24, 2013
 * @brief This executable shows workspace control with a spacenav in the dart/grip environment.
 * NOTE: Although the spacenav input is interpreted as a workspace velocity, we first create a 
 * workspace reference from it and then compute xdot back. The motivation is under application such
 * as liberty and compliance plays with workspace reference configurations.
 */

#define protected public
#define private public

#include "simTab.h"
#include "GRIPApp.h"
#include <math/UtilsRotation.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include "workspace.h"
#include "sensors.h"
#include "safety.h"

using namespace Krang;
using namespace Eigen;
using namespace dynamics;

somatic_d_t daemon_cx;
ach_channel_t spacenav_chan;
SkeletonDynamics* robot;
VectorXd homeConfig(7);			///< Home configuration for the left arm

// workspace stuff
Krang::WorkspaceControl* ws;
VectorXd nullspace_dq_ref;
bool myDebug;

/* ********************************************************************************************* */
/// Configurations for workspace control
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const Eigen::VectorXd NULLSPACE_Q_REF_INIT = 
		(VectorXd(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.25;
const double SPACENAV_TRANSLATION_GAIN = 0.25; 
const double COMPLIANCE_GAIN = 1.0 / 750.0;

std::vector <Vector6d> data;

/* ********************************************************************************************* */
/// Given a desired workspace translation vel. for the elbow, returns the jointspace velocities for 
/// the top 3 joints using inverse Jacobian (and other 4 zeroes). This velocity would be computed
/// simply as difference of cur and ref position in the case of liberty and mouse/grip combo, but
/// for spacenav we might want to compute the xdot based on task-space goals such as keeping the 
/// elbow level.
VectorXd elbowQdot (const VectorXd xdotElbow, Side side) {
	
	if(myDebug) DISPLAY_VECTOR(xdotElbow);

	// Get the linear jacobian
	kinematics::BodyNode* elbow = robot->getNode(side == LEFT ? "L3" : "R3");
	MatrixXd fullJ = elbow->getJacobianLinear();
	if(myDebug) DISPLAY_MATRIX(fullJ);
	MatrixXd Jlin = elbow->getJacobianLinear().topRightCorner<3,3>(); //topLeftCorner<3,3>();
	if(myDebug) DISPLAY_MATRIX(Jlin);
	
	// Compute the inverse of linear jacobian
	MatrixXd JlinInv = Jlin;
	aa_la_inv(3, JlinInv.data());
	if(myDebug) std::cout << "JlinInv:\n" << JlinInv << std::endl;

	// Compute the qdotElbow
	VectorXd qdotElbow = Jlin.transpose() * xdotElbow;
	if(myDebug) std::cout << "qdotElbow before magn: " << qdotElbow.transpose() << std::endl;
	
	// Limit the magnitude of the velocity
	double magn = qdotElbow.norm();
	if(magn > 0.5) qdotElbow *= (0.5 / magn);
	else if(magn < 0.0005) qdotElbow = VectorXd::Zero(3);
	if(myDebug) DISPLAY_VECTOR(qdotElbow);

	// Fill the last 4 values with zeroes to get a velocity for the entire arm
	VectorXd qdotElbowFull = VectorXd::Zero(7);
	qdotElbowFull.topLeftCorner<3,1>() = qdotElbow;
	if(myDebug) DISPLAY_VECTOR(qdotElbowFull);
	return qdotElbowFull;
}

/* ********************************************************************************************* */
/// Returns the joint angles for the top two joints which control the location of the elbow
void getElbowAngles (const Vector3d& ref, double imuWaist, double& q0, double& q1) {

	// Get the reference in the left arm base joint frame
	double x = cos(imuWaist) * ref(0) - sin(imuWaist) * ref(2);
	double y = ref(1);
	double z = sin(imuWaist) * ref(0) + cos(imuWaist) * ref(2);

	// Compute the angles
	q0 = -atan2(-z, x);
	q1 = atan2(-x * cos(q0) - z * sin(q0), y);
	q0 += M_PI_2;
}

/* ********************************************************************************************* */
/// Gets the workspace velocity input from the spacenav, updates a workspace reference position
/// and moves the left arm end-effector to that position
void Timer::Notify() {

	static int c_ = 0;
	myDebug = ((c_++ % 5) == 0);
	ws->debug = myDebug;
	if(myDebug) std::cout << "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << std::endl;

	// ============================================================================
	// Update the time and spacenav reading, and compute the input velocities

	// Update times
	static double time_last = aa_tm_timespec2sec(aa_tm_now());
	double time_now = aa_tm_timespec2sec(aa_tm_now());
	double time_delta = time_now - time_last;
	time_last = time_now;

	// Get the workspace velocity input from the spacenav
	VectorXd spacenav_input (6);
	static Eigen::VectorXd last_spacenav_input = Eigen::VectorXd::Zero(6);
	static int badSpaceNavCtr = 0;
	if(!getSpaceNav(&spacenav_chan, spacenav_input)) 
		spacenav_input = (badSpaceNavCtr++ > 20) ? VectorXd::Zero(6) : last_spacenav_input;
	else badSpaceNavCtr = 0;
	last_spacenav_input = spacenav_input;

	bool readFromFile = true; 
	static size_t dataCounter = 0;
	if(readFromFile) spacenav_input = data[std::min(data.size() - 1, dataCounter++)];
	else data.push_back(spacenav_input);

	// Get the joint angles we want to bias towards
	Eigen::VectorXd q = robot->getConfig(*ws->arm_ids);
	MatrixXd Tg2 = mWorld->getSkeleton("g2")->getNode("link_0")->getWorldTransform();
	MatrixXd TL2 = mWorld->getSkeleton("Krang")->getNode("L1")->getWorldTransform();
	Vector3d Tref = Tg2.topRightCorner<3,1>() - TL2.topRightCorner<3,1>();
	double offset = -3.45 + 2.81;
	VectorXd qElbowRef = q;
	//getElbowAngles(Tref, offset, qElbowRef(0), qElbowRef(1));
	qElbowRef(0) = 0.95;
	qElbowRef(3) = -0.5;
	nullspace_dq_ref = (qElbowRef - q).normalized();

	// Compute the desired jointspace velocity from the inputs (no ft sensor)
	Eigen::VectorXd qdot_jacobian;
	VectorXd ft = VectorXd::Zero(6);
	ws->update(spacenav_input, ft, nullspace_dq_ref, time_delta, qdot_jacobian);

	// Visualize the reference position
	VectorXd refConfig = transformToEuler(ws->Tref, math::XYZ);
	mWorld->getSkeleton("g1")->setConfig(dart_root_dof_ids, refConfig);

	// ============================================================================
	// Threshold the input jointspace velocity for safety and send them

	// Scale the values down if the norm is too big or set it to zero if too small
	double magn = qdot_jacobian.norm();
	if(magn > 0.5) qdot_jacobian *= (0.5 / magn);
	else if(magn < 0.05) qdot_jacobian = VectorXd::Zero(7);
	if(myDebug) DISPLAY_VECTOR(qdot_jacobian);

	// Avoid joint limits - set velocities away from them as we get close
	Eigen::VectorXd qdot_avoid(7);
	if(myDebug) DISPLAY_VECTOR(q);
	computeQdotAvoidLimits(robot, *(ws->arm_ids), q, qdot_avoid);
	if(myDebug) DISPLAY_VECTOR(qdot_avoid);

	// Add our qdots together to get the overall movement
	Eigen::VectorXd qdot_apply = qdot_avoid + qdot_jacobian;
	if(myDebug) DISPLAY_VECTOR(qdot_apply);
	
	// Integrate the qdot we want to apply
	q += 0.03 * qdot_apply;
	robot->setConfig(left_arm_ids, q);

	// Visualize the scene
	viewer->DrawGLScene();
	Start(0.005 * 1e4);
}

/* ********************************************************************************************* */
void readData () {

	std::ifstream file ("data");
	assert(file.is_open());
	char line [1024];
	while(file.getline(line, 1024)) {
		std::stringstream stream(line, std::stringstream::in);
		double newDouble;
		VectorXd newData (6);
		size_t i = 0;
		while ((stream >> newDouble)) (newData)(i++) = newDouble;
		data.push_back(newData);
		std::cout << newData.transpose() << std::endl;
	}
}


/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size,
		long style) : GRIPTab(parent, id, pos, size, style) {

	// ============================================================================
	// Initialize grip stuff

  wxBoxSizer* sizerFull = new wxBoxSizer(wxHORIZONTAL);
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(-0.3, 0.0, -0.8);
	Matrix3d rotM; 
	rotM << -0.8459, 0.0038246, 0.5332,0.000573691,-0.9999,0.008082, 0.533265, 0.00714295, 0.845918;
	viewer->camRotT = rotM;
	viewer->UpdateCamera();
	SetSizer(sizerFull);
	frame->DoLoad("../../common/scenes/05-World-Teleop.urdf");
	robot = mWorld->getSkeleton("Krang");

	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->Start(1);	

	// ============================================================================
	// Initialize robot stuff

	readData();

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "05-spacenav";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the spacenav channel
	somatic_d_channel_open(&daemon_cx, &spacenav_chan, "spacenav-data", NULL);

	// Manually set the initial arm configuration for the left arm
	homeConfig <<  0.97, -0.589,  0.000, -1.339,  0.000, -0.959, -1.000;
	robot->setConfig(left_arm_ids, homeConfig);

	// Also, set the imu and waist angles
	std::vector <int> imuWaist_ids;
	imuWaist_ids.push_back(5);
	imuWaist_ids.push_back(8);
	Vector2d imuWaist (3.45, 2.81);
	robot->setConfig(imuWaist_ids, imuWaist);

  // Set the position of the reference indicator, g2, to elbow position
  kinematics::BodyNode* elbow = robot->getNode("L3");
  MatrixXd Tcur = elbow->getWorldTransform();
  mWorld->getSkeleton("g2")->setConfig(dart_root_dof_ids, transformToEuler(Tcur, math::XYZ));

	// Set up the workspace controllers
	ws = new WorkspaceControl(robot, LEFT, K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
		SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN, COMPLIANCE_GAIN);

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

}

/* ********************************************************************************************* */
SimTab::~SimTab() {

	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Clean up the daemon resources
	somatic_d_destroy(&daemon_cx);

	for(size_t i = 0; i < data.size(); i++)
		std::cout << data[i].transpose() << std::endl;
}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() {}
void SimTab::OnButton(wxCommandEvent &evt) {}
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
		tabView->AddPage(new SimTab(tabView), wxT("Teleop"));
	}
};

IMPLEMENT_APP(mainApp)


