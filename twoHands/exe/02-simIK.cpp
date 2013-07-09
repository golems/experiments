/**
 * @file 03-simIK.cpp
 * @author Can Erdogan
 * @date May 12, 2013
 * @brief Chooses random configurations in the nearby vicinity of the robot and sees if it can 
 * perform inverse kinematics for the left arm. If the robot can move there, it would move. 
 * Otherwise, it would put its arm in the zero position.
 */

#define protected public
#define private public

#include "simTab.h"
#include "ik.h"
#include "GRIPApp.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;

bool rightArm = 0;
bool bothArms = 1;

/* ********************************************************************************************* */
/// Returns the IK goals for turning a lever to lift a box. The green arrow is at the fulcrum
/// of the lever.
void armGoalsLeverDown (const Matrix4d& Twee, Matrix4d& TweeL, Matrix4d& TweeR) {

	// Decide on the frame for the left hand
	TweeL.topRightCorner<4,1>() -= Twee.topLeftCorner<4,1>().normalized() * 0.20;
	TweeL.topLeftCorner<3,3>() =  Twee.topLeftCorner<3,3>() *
		AngleAxis<double>(-M_PI_2, Vector3d(0.0, 0.0, 1.0)).matrix(); 


	// Decide on the frame for the right hand given the left hand
	TweeR = TweeL;
	TweeR.topRightCorner<4,1>() -= TweeL.block<4,1>(0,1).normalized() * 0.20;
}

/* ********************************************************************************************* */
/// Returns the I.K. goals for turning a valve where the green arrow is at the center of the valve
/// Basically, we want each end-effector to look towards the green arrow at some distance .08
void armGoalsValveSideGrip (const Matrix4d& Twee, Matrix4d& TweeL, Matrix4d& TweeR) {

	// Decide on the frame for the left hand
	TweeL.topRightCorner<4,1>() += Twee.topLeftCorner<4,1>().normalized() * 0.12;
	TweeL.topLeftCorner<3,3>() =  Twee.topLeftCorner<3,3>() *
		AngleAxis<double>(M_PI_2, Vector3d(0.0, 0.0, 1.0)).matrix() * 
		AngleAxis<double>(M_PI_2, Vector3d(1.0, 0.0, 0.0)).matrix();

	// Decide on the frame for the right hand given the left hand
	TweeR = TweeL;
	TweeR.topRightCorner<4,1>() -= TweeL.block<4,1>(0,2).normalized() * 0.24;
	TweeR.topLeftCorner<3,3>() =  TweeL.topLeftCorner<3,3>() *
		AngleAxis<double>(-M_PI, Vector3d(1.0, 0.0, 0.0)).matrix();
}

/* ********************************************************************************************* */
/// Returns the I.K. goals for a rotation motion where the green arrow is at the center. We want
/// the side of the end-effectors to be towards the arrow and front looking down.
void armGoalsValveDownGrip (const Matrix4d& Twee, Matrix4d& TweeL, Matrix4d& TweeR) {

	// Decide on the frame for the left hand
	TweeL.topRightCorner<4,1>() += Twee.topLeftCorner<4,1>().normalized() * 0.20;
	TweeL.topLeftCorner<3,3>() = Twee.topLeftCorner<3,3>() *
		AngleAxis<double>(M_PI_2, Vector3d(0.0, 0.0, 1.0)).matrix() * 
		AngleAxis<double>(-M_PI_2, Vector3d(0.0, 1.0, 0.0)).matrix();

	// Decide on the frame for the right hand given the left hand
	TweeR = TweeL;
	TweeR.topRightCorner<4,1>() += TweeL.block<4,1>(0,1).normalized() * 0.40;
//	TweeR.topLeftCorner<3,3>() =  TweeL.topLeftCorner<3,3>() *
//		AngleAxis<double>(M_PI, Vector3d(1.0, 0.0, 0.0)).matrix();
}

/* ********************************************************************************************* */
void armGoalsStick (const Matrix4d& Twee, Matrix4d& TweeL, Matrix4d& TweeR) {

	// Decide on the frame for the left hand
	TweeL = Twee;
	TweeL.topRightCorner<4,1>() += Twee.block<4,1>(0,1).normalized() * 0.28;

	TweeR = TweeL;
	TweeR.topRightCorner<4,1>() -= TweeL.block<4,1>(0,1).normalized() * 0.56;
}

/* ********************************************************************************************* */
/// Performs I.K. to both arms
bool bothArmsIK (SkeletonDynamics* robot, const Matrix4d& Twee) {

	// Get the end-effector goals for a specific task
	Matrix4d TweeL = Twee, TweeR = Twee;
	armGoalsLeverDown(Twee, TweeL, TweeR);
	pmr(TweeL);
	pmr(TweeR);

	// Get the IK for the left arm
	bool successL = singleArmIK(mWorld, robot, TweeL, false);

	// Get the IK for the second arm for a slightly left location
	bool successR = singleArmIK(mWorld, robot, TweeR, true);
	return successL && successR;
}

/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left 
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {
	
	cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << endl;
	// =======================================================================
	// Get the relative goal in the bracket frame

	// Get the goal config
	vector <int> obj_dof;
	for(size_t i = 0; i < 6; i++) obj_dof.push_back(i);
	VectorXd goal = mWorld->getSkeleton(0)->getConfig(obj_dof);

	// Create the end-effector frame transformation
	Transform <double, 3, Affine> Twee;
	Vector3d euler (goal(5), goal(4), goal(3));
	Twee.linear() = math::eulerToMatrix(euler, math::XYZ);
	Twee.translation() = goal.segment(0,3);

	// =======================================================================
	// Compute the IK and draw the result if applicable

	// Perform I.K. either for a single arm or both arms
	bool success = false;
	SkeletonDynamics* robot = mWorld->getSkeleton(2);
	if(bothArms) success = bothArmsIK(robot, Twee.matrix());
	else success = singleArmIK (mWorld, robot, Twee.matrix(), rightArm);

	// Display the end-effector position and the arm configuration (and draw floor red if fail)
	viewer->backColor = Vector3d(success ? 0.0 : 1.0, 0.0, 0.0);
	viewer->setClearColor();
	viewer->DrawGLScene();
	
	// Restart the timer for the next start
	Start(0.1 * 1e3);	
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
		long style) : GRIPTab(parent, id, pos, size, style) {

	// Create the sizer
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
 	// Create the sizer that controls the tab panel
	wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
	
	// Set the camera zoom
	viewer->camRadius = 2.0;
	viewer->worldV += Vector3d(0.0, -0.0, -0.8);
	viewer->UpdateCamera();

	// Set the full sizer as the sizer of this tab
	SetSizer(sizerFull);

	// Load the schunk scene automatically
	frame->DoLoad("../../common/scenes/03-World-IK.urdf");

	// Set the imu and waist values
	vector <int> imuWaist_ids; 
	imuWaist_ids.push_back(5);	
	imuWaist_ids.push_back(8);	
	mWorld->getSkeleton(2)->setConfig(imuWaist_ids, 
		Vector2d(1.857 - 3.0 * M_PI_2, (150.0 / 180.0) * M_PI));

	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->world = mWorld;
	timer->Start(100);	
}

/* ********************************************************************************************* */
void SimTab::GRIPEventSimulationBeforeTimestep() {}

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
		tabView->AddPage(new SimTab(tabView), wxT("Inverse Kinematics"));
	}
};

IMPLEMENT_APP(mainApp)
