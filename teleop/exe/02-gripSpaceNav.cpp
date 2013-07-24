/**
 * @file 05-spacenav.cpp
 * @author Can Erdogan, Saul Reynolds-Haertle, Jon Scholtz
 * @date July 24, 2013
 * @brief This executable shows workspace control with a spacenav in the dart/grip environment.
 * NOTE: Although the spacenav input is interpreted as a workspace velocity, we first create a 
 * workspace reference from it and then compute xdot back. The motivation is under application such
 * as liberty and compliance plays with workspace reference configurations.
 */

#define pv(a) std::cout << #a << ": " << fix((a).transpose()) << std::endl
#define pmr(a) std::cout << #a << ":\n " << fix((a)) << std::endl

#define protected public
#define private public

#include "simTab.h"
#include "GRIPApp.h"
#include <math/UtilsRotation.h>

#include "util.h"
#include "initModules.h"
#include "motion.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;

somatic_d_t daemon_cx;
ach_channel_t spacenav_chan;

SkeletonDynamics* robot;
vector <int> left_arm_ids;  ///< The indices to the Krang dofs in dart

VectorXd homeConfig(7);			///< Home configuration for the left arm
Matrix4d Tref;							///< The reference pos/ori for the left end-effector

/* ******************************************************************************************** */
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < 1e-5) mat2(i,j) = 0.0;
	return mat2;
}

/* ********************************************************************************************* */
/// Returns the 6 axes values from the spacenav
bool getSpaceNav(VectorXd& config) {

	// Get joystick data
	int r = 0;
	config = VectorXd::Zero(6);
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK(r, somatic__joystick,
			&protobuf_c_system_allocator, 4096, &spacenav_chan);
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return false;

	// Set the data to the input reference
	Somatic__Vector* x = js_msg->axes;
	config << -x->data[1], -x->data[0], -x->data[2], -x->data[4], -x->data[3], -x->data[5];

	// Free the liberty message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
	return true;
}

/* ********************************************************************************************* */
/// Scales the parameter spacenav input and computes the new reference configuration
void updateReference (VectorXd& spacenav_input) {

	// Scale the translation and orientation components of displacement; and make a matrix for it
	spacenav_input.topLeftCorner<3,1>() *= 0.02;
	spacenav_input.bottomLeftCorner<3,1>() *= 0.06;
	Matrix4d xdotM = eulerToTransform(spacenav_input, math::XYZ);
	
	// Compute the displacement in the end-effector frame with a similarity transform
	Matrix4d R = Tref;
	R.topRightCorner<3,1>().setZero();
	Matrix4d Tdisp = R.inverse() * xdotM * R;

	// Update the reference position for the end-effector with the given workspace velocity
	Tref = Tref * Tdisp;
}

/* ********************************************************************************************* */
/// Returns the workspace velocity, xdot, from the reference, Tref, using the current arm
/// configuration
void getXdotFromXref (VectorXd& xdot) {

	// Get the current end-effector transform and also, just its orientation 
	Matrix4d Tcur = robot->getNode("lGripper")->getWorldTransform();
	Matrix4d Rcur = Tcur;
	Rcur.topRightCorner<3,1>().setZero();

	// Apply the similarity transform to the displacement between current transform and reference
	Matrix4d Tdisp = Tcur.inverse() * Tref;
	Matrix4d xdotM = Rcur * Tdisp * Rcur.inverse();
	xdot = transformToEuler(xdotM, math::XYZ);
}

/* ********************************************************************************************* */
/// Compute qdot with the dampened inverse Jacobian with nullspace projection
/// Important constants are for the null space gain, the dampening gain and the second goal pos.
void getQdot (const VectorXd& xdot, VectorXd& qdot) {

	// Set the parameter constants
	static const double dampGain = 0.005;
	static const double qdotRefDt = 0.0;
	static const VectorXd qRef = (VectorXd(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();

	// Get the Jacobian for the left end-effector
	kinematics::BodyNode* ee = robot->getNode("lGripper");
	MatrixXd Jlin = ee->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = ee->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian with dampening
	MatrixXd Jt = J.transpose(), JJt = J * Jt;
	for (int i=0; i < JJt.rows(); i++) JJt(i,i) += dampGain;

	// Compute the reference joint velocities for nullspace projection
	VectorXd q = robot->getConfig(left_arm_ids);
	VectorXd qDotRef = (q - qRef) * qdotRefDt;
	
	// Compute the qdot using the reference joint velocity and the reference position 
	MatrixXd Jinv = Jt * JJt.inverse();
	MatrixXd JinvJ = Jinv*J;
	MatrixXd I = MatrixXd::Identity(7,7);
	qdot = Jinv * xdot + (I - JinvJ) * qDotRef;
}

/* ********************************************************************************************* */
/// Gets the workspace velocity input from the spacenav, updates a workspace reference position
/// and moves the left arm end-effector to that position
void Timer::Notify() {

	// Get the workspace velocity input from the spacenav
	VectorXd spacenav_input (6);
	bool result = false;
	while(!result) result = getSpaceNav(spacenav_input);

	// Update the reference configuration with the spacenav input and visualize it
	updateReference(spacenav_input);
	mWorld->getSkeleton("g1")->setConfig(dartRootDofOrdering, transformToEuler(Tref, math::XYZ));
	pv(spacenav_input);

	// Get xdot from the reference configuration
	VectorXd xdot;
	getXdotFromXref(xdot);

	// Compute qdot with the dampened inverse Jacobian with nullspace projection
	VectorXd qdot;
	getQdot(xdot, qdot);

	// Apply the joint velocities
	Eigen::VectorXd q = robot->getConfig(left_arm_ids);
	q += qdot * 0.03;
	robot->setConfig(left_arm_ids, q);

	// Visualize the scene
	viewer->DrawGLScene();
	Start(0.005 * 1e4);
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

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "05-spacenav";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the spacenav channel
	somatic_d_channel_open(&daemon_cx, &spacenav_chan, "spacenav-data", NULL);

	// Manually set the initial arm configuration for the left arm
	for(int i = 11; i < 24; i+=2) left_arm_ids.push_back(i);
	homeConfig <<  1.102, -0.589,  0.000, -1.339,  0.000, -0.959, -1.000;
	robot->setConfig(left_arm_ids, homeConfig);

	// Also, set the imu and waist angles
	vector <int> imuWaist_ids;
	imuWaist_ids.push_back(5);
	imuWaist_ids.push_back(8);
	Vector2d imuWaist (3.45, 2.81);
	robot->setConfig(imuWaist_ids, imuWaist);

	// Initialize the "initial" reference configuration for the end-effector with current
	Tref = robot->getNode("lGripper")->getWorldTransform();

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


