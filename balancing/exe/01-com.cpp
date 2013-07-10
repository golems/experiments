/**
 * @file 01-com.cpp
 * @author Can Erdogan, Peng Hou, Munzir Zafar
 * @date July 08, 2013
 * @brief This executable shows how to estimate the center of mass of the Krang and visualizes
 * the total center of mass and individual body nodes center of masses in grip with the data
 * read from the joints and imu sensor. 
 * Note if you want to move the joints, please use the home utility.
 */

#define protected public
#define private public

#define RAD2DEG RAD2DEG	// to deal with a define bug in grip which clashes with pcl
#define DEG2RAD DEG2RAD

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <filter.h>

#include <Eigen/Dense>
#include <iostream>

#include "GRIPApp.h"
#include "simTab.h"
#include "helpers.h"

using namespace std;
using namespace Eigen;
using namespace dynamics;

#define NUM_ROWS 480
#define NUM_COLUMNS 640

#define getMotorMessage(x) (SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, \
	&protobuf_c_system_allocator, 1024, &x, &abstime))

/* ******************************************************************************************** */
// Communication and filtering

filter_kalman_t *kf;					///< the kalman filter to smooth the imu readings
somatic_d_t daemon_cx;				///< the daemon properties
ach_channel_t imuChan;				///< the state channel to listen to imu data
ach_channel_t waistChan;			///< the state channel for the waist module
ach_channel_t leftArmChan;		///< the state channel for the left arm modules
ach_channel_t rightArmChan;		///< the state channel for the right arm modules

SkeletonDynamics* robot;			///< the robot representation in dart
struct timespec t_now, t_prev;///< for timing each iteration

wxStaticText* text;						///< the text box to show the com value

/* ********************************************************************************************* */
/// Gets the data from the channels
void getData (filter_kalman_t* kf, double dt) {

	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);

	// Get the data from the motors
	int r;
	Somatic__MotorState *waist, *leftArm, *rightArm;
	assert(((waist = getMotorMessage(waistChan)) != NULL) && "Waist call failed");
	assert(((leftArm = getMotorMessage(leftArmChan)) != NULL) && "leftArm call failed");
	assert(((rightArm = getMotorMessage(rightArmChan)) != NULL) && "rightArm call failed");
	
	// Get the data from imu 
	double imu, imuSpeed;
	getImu(&imuChan, imu, imuSpeed, dt, kf);

	// Set the imu and waist angles, averaging the waist angle values
	double waist_val = (waist->position->data[0] - waist->position->data[1]) / 2.0;
	Vector2d imuWaist_vals (-imu + M_PI_2, waist_val);
	robot->setConfig(imuWaist_ids, imuWaist_vals);
	
	// Update the robot state
	Vector7d larm_vals = eig7(leftArm->position->data), rarm_vals = eig7(rightArm->position->data);
	robot->setConfig(left_arm_ids, larm_vals);
	robot->setConfig(right_arm_ids, rarm_vals);

	// Clean up the messages
	somatic__motor_state__free_unpacked(leftArm, &protobuf_c_system_allocator);
	somatic__motor_state__free_unpacked(rightArm, &protobuf_c_system_allocator);
	somatic__motor_state__free_unpacked(waist, &protobuf_c_system_allocator);
}

/* ********************************************************************************************* */
/// Picks a random configuration for the robot, moves it, does f.k. for the right and left 
/// end-effectors, places blue and green boxes for their locations and visualizes it all
void Timer::Notify() {

	// Get the current time and compute the difference
	t_now = aa_tm_now();
	double dt = (double) aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));
	t_prev = t_now;

	// Get the data from ach channels
	getData(kf, dt);

	// Reflect the changes in the dart kinematics and update the viewer
	viewer->DrawGLScene();

	// Get the center of mass and balancing angle (theta), and write it to the gui
	char buf [256];
	Vector3d com = robot->getWorldCOM();
	double theta = atan2(com(0), com(2));
	sprintf(buf, "com (cm): (%lf, %lf, %lf)\n\nbalancing angle (deg): %lf", 100.0 * com(0), 
		100.0 * com(1), 100.0 * com(2), (theta / M_PI) * 180.0);
	text->SetLabel(wxString(buf, wxConvUTF8));
		
	
	// Clean up the daemon memory
	aa_mem_region_release(&daemon_cx.memreg);
	
	// Restart the timer for the next start
	Start(0.005 * 1e4);	
}

/* ********************************************************************************************* */
SimTab::SimTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, 
		long style) : GRIPTab(parent, id, pos, size, style) {

	// ============================================================================
	// Initialize grip stuff

  sizerFull = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
	viewer->camRadius = 3.0;
	viewer->worldV += Vector3d(0.0, 0.0, -0.7);
	viewer->UpdateCamera();
	SetSizer(sizerFull);
	frame->DoLoad("../../common/scenes/01-World-Robot.urdf");
	robot = mWorld->getSkeleton(0);

	// Create a text box to show the center of mass
 	text = new wxStaticText(this, wxID_ANY, wxT("(0.0, 0.0, 0.0)"), wxPoint(15, 15));
  wxFont font(16, wxDEFAULT, wxNORMAL, wxBOLD);
  text->SetFont(font);

	// Create the timer to notify the function that draws the robot at multiple configurations
	timer = new Timer();
	timer->Start(1);	

	// ============================================================================
	// Initialize robot stuff

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-com";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the channels to the sensors
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	somatic_d_channel_open(&daemon_cx, &leftArmChan, "llwa-state", NULL);
	somatic_d_channel_open(&daemon_cx, &rightArmChan, "rlwa-state", NULL);

	// Initialize kalman filter for the imu and set the measurement and meas. noise matrices
	kf = new filter_kalman_t;
	filter_kalman_init(kf, 2, 0, 2);
	kf->C[0] = kf->C[3] = 1.0;
	kf->Q[0] = kf->Q[3] = 1e-3;

	// Start the timing
	t_prev = aa_tm_now();

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

/* ********************************************************************************************* */
SimTab::~SimTab() {
	
	// Send the stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
  
	// Destroy the channel and daemon resources
	somatic_d_channel_close(&daemon_cx, &imuChan);
	somatic_d_channel_close(&daemon_cx, &waistChan);	 
	somatic_d_channel_close(&daemon_cx, &leftArmChan); 
	somatic_d_channel_close(&daemon_cx, &rightArmChan);
	somatic_d_destroy(&daemon_cx);
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
		tabView->AddPage(new SimTab(tabView), wxT("Center of mass"));
	}
};

IMPLEMENT_APP(mainApp)