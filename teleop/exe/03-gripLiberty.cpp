/**
 * @file 03-gripLiberty.cpp
 * @author Can Erdogan
 * @date June 18, 2013
 * @brief This file demonstrates how to visualize the motion of the arms in grip.
 * NOTE Although I wanted to change this, we had to make the GRIP/wxWidget the main program
 * (the surrounding thread) and send data within a timer... This could be bad if for some reason
 * visualization halts and we want to stop the arms right then.
 */


#define protected public
#define private public

#include <GRIPApp.h>
#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <math/UtilsRotation.h>

#include "simTab.h"

#include "kore.h"
#include "util.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t liberty_chan;

SkeletonDynamics* robot;
vector <int> left_arm_ids;  ///< The indices to the Krang dofs in dart

VectorXd homeConfig(7);			///< Home configuration for the left arm
Matrix4d Tref;							///< The reference pos/ori for the left end-effector
Matrix4d Tlst;              ///< The current configuration of the sensor in the liberty frame
Matrix4d Twg0;							///< The initial configuration of the gripper in the world frame
Matrix4d Tls0;							///< The initial configuration of the sensor in the liberty frame


/* ********************************************************************************************* */
/// Returns the values of the first liberty sensor
bool getLiberty(VectorXd& config) {

	// Get the data
	int r = 0;
	config.setZero(); 
	Somatic__Liberty *l_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__liberty,
													   &protobuf_c_system_allocator, 1024, &liberty_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (l_msg == NULL)) return false;

	// Set the values for the position
	Somatic__Vector* s1 = l_msg->sensor1;
	config << s1->data[0], -s1->data[1], -s1->data[2], 0.0, 0.0, 0.0;

	// Convert from a quaternion to rpy representation
	Quaternion <double> oriQ (s1->data[6], s1->data[3], s1->data[4], s1->data[5]);
	Matrix3d oriM = oriQ.matrix();
	Vector3d oriE = math::matrixToEuler(oriM, math::XYZ);
	config.bottomLeftCorner<3,1>() << -oriE[2], -oriE[1], oriE[0];
	return true;
}

/* ********************************************************************************************* */
/// Takes a position from the liberty and uses it to compute a new
/// reference configuration.
void updateReference(VectorXd& liberty_input) {

	Tlst = Krang::eulerToTransform(liberty_input, math::XYZ);
	DISPLAY_MATRIX(Tlst);
//	Tref = Tlst * Tls0.inverse() * Twg0;
	DISPLAY_MATRIX(Tref);
	Vector3d disp = Tlst.topRightCorner<3,1>() - Tls0.topRightCorner<3,1>();
	Tref.topRightCorner<3,1>() = Twg0.topRightCorner<3,1>() + disp;
	Tref.topLeftCorner<3,3>() = Twg0.topLeftCorner<3,3>();
	Tref(3,3) = 1.0;

	Matrix3d Rlst = Tlst.topLeftCorner<3,3>();
	Matrix3d Rls0 = Tls0.topLeftCorner<3,3>();
	Matrix3d Rwg0 = Twg0.topLeftCorner<3,3>();
	Tref.topLeftCorner<3,3>() = Rlst * Rls0.inverse() * Rwg0;
/*
	// Compute the current configuration of the sensor in the liberty frame
	Tlst = eulerToTransform(liberty_input, math::XYZ);
	pmr(Tlst);

	// Compute the displacement between the initial sensor and gripper configurations
	Matrix4d Toffset = Twg0 * Tls0.inverse();
	Toffset.topLeftCorner<3,3>() = Matrix3d::Identity();

	// Compute the reference 
	pmr(Toffset);
	Tref = Toffset * Tlst;
	pmr(Tref);
*/
}

/* ********************************************************************************************* */
/// Returns the workspace velocity, xdot, from the reference, Tref, using the current arm
/// configuration
void getXdotFromXref (VectorXd& xdot) {

	// Get the current end-effector transform and also, just its orientation 
	Matrix4d Tcur = robot->getNode("lGripper")->getWorldTransform();
	pmr(Tcur);
	Matrix4d Rcur = Tcur;
	Rcur.topRightCorner<3,1>().setZero();

	// Apply the similarity transform to the displacement between current transform and reference
	Matrix4d Tdisp = Tcur.inverse() * Tref;
	Matrix4d xdotM = Rcur * Tdisp * Rcur.inverse();
	xdot = Krang::transformToEuler(xdotM, math::XYZ);
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
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left 
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {

	cout << "\nvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << endl;

	// Get the liberty info
	VectorXd liberty_input (6);
	bool result = false;
	while(!result) result = getLiberty(liberty_input);

	// Set the initial values
	static int c_ = 0;
	if(c_ == 0) {
		Twg0 = robot->getNode("lGripper")->getWorldTransform();
		Tls0 = Krang::eulerToTransform(liberty_input, math::XYZ);
	}
	c_++;

	// Update the reference configuration with the liberty input
	updateReference(liberty_input);

	// Get xdot from the reference configuration
	VectorXd xdot;
	getXdotFromXref(xdot);
	pv(xdot);

	// Compute qdot with the dampened inverse Jacobian with nullspace projection
	VectorXd qdot;
	getQdot(xdot, qdot);
	pv(qdot);

	// Apply the joint velocities
	Eigen::VectorXd q = robot->getConfig(left_arm_ids);
	q += qdot * 0.03;
	pv(q);
	robot->setConfig(left_arm_ids, q);

	// Visualize the scene
	viewer->DrawGLScene();
	Start(0.005 * 1e4);
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
			   long style) : GRIPTab(parent, id, pos, size, style) {

	// ============================================================================
	// Create user interface
	wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
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
	// Initialize the ach stuff

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "03-gripLiberty";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the liberty client
	somatic_d_channel_open(&daemon_cx, &liberty_chan, "liberty", NULL);

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

	// clean up daemon resources
	somatic_d_destroy(&daemon_cx);
}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() {}

/* ********************************************************************************************* */
// Handler for events

BEGIN_EVENT_TABLE(SimTab, wxPanel)
// EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimTab::OnButton)
// EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimTab::OnSlider)
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
