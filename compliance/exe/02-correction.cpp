/**
 * @file correction.cpp
 * @author Can Erdogan
 * @date June 14, 2013
 * @brief This file demonstrates how to compute the estimate what the raw force/torque reading 
 * should be when there are no external forces (only the gripper weight) and the offset that
 * needs to be decreased from future raw values.
 * The output is that as we move the arm around, we should see no change in the idealized values.
 * See the FTEstimation report at @thebrain:/home/git/krang/Reports.
 * Note that the F/T readings are in Nm's (Newton-meters).
 */

#include "helpers.h"
#include "initModules.h"
#include "motion.h"
#include "kinematics.h"

using namespace std;
using namespace kinematics;

#define pv(x) cout << #x << ": " << (x).transpose() << endl;

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;
Vector6d offset;							///< the offset we are going to decrease from raw readings

/* ******************************************************************************************** */
/// Returns the representation of the end-effector frame in the base frame
void forwardKinematics (MatrixXd& Tbee) {

	// Create the DH table for the arm
	double T [7][4] = {
		{0.0, -M_PI_2, -L4, llwa.pos[0]},
		{0.0, M_PI_2, 0.0, llwa.pos[1]},
		{0.0, -M_PI_2, -L5, llwa.pos[2]},
		{0.0, M_PI_2, 0.0, llwa.pos[3]},
		{0.0, -M_PI_2, -L6, llwa.pos[4]},
		{0.0, M_PI_2, 0.0, llwa.pos[5]},
		{0.0, 0.0, -L7-L8, llwa.pos[6]}};

	// Loop through the joints and aggregate the transformations multiplying from left
	Tbee = MatrixXd::Identity(4,4);
	for(size_t i = 0; i < 7; i++) 
		Tbee *= dh(T[i][0], T[i][1], T[i][2], T[i][3]);		
}

/* ******************************************************************************************** */
void computeOffset (const Vector6d& raw) {

	// Set the vector from the sensor origin to the gripper center of mass
	static const Vector3d s2com (0.0, 0.0, 0.02);

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the bracket frame and the sensor frame. We assume that the ee
	// frame is 180 rotation around y and 90 rotation around x away from the 7th module;
	MatrixXd Tbee;
	forwardKinematics(Tbee);
	Matrix3d Rees = (AngleAxis <double> (M_PI, Vector3d(0.0, 1.0, 0.0)) * 
                  AngleAxis <double> (M_PI_2, Vector3d(0.0, 0.0, 1.0))).matrix();
	Matrix3d R = Rees.transpose() * Tbee.topLeftCorner<3,3>().transpose();

	// Create the wrench with the computed rotation to change the frame from the bracket to the sensor
	Matrix6d pSsensor_bracket = MatrixXd::Identity(6,6); 
	pSsensor_bracket.topLeftCorner<3,3>() = R;
	pSsensor_bracket.bottomRightCorner<3,3>() = R;
	
	// Get the weight vector (note that we use the bracket frame for gravity so towards -y)
	static const double eeMass = 0.169;	// kg - ft extension
	Vector6d weightVector_in_bracket;
	weightVector_in_bracket << 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d expectedFT = pTcom_sensor * pSsensor_bracket * weightVector_in_bracket;

	// Compute the difference between the actual and expected f/t values
	offset = expectedFT - raw;
}

/* ********************************************************************************************* */
void init (){

	/// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
	system("killall -s 9 netcanftd");
	usleep(20000);
	system("netcanftd -v -d -I lft -b 2 -B 1000 -c llwa_ft -r");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-gripperWeight";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the left arm
	initArm(daemon_cx, llwa, "llwa");
	somatic_motor_update(&daemon_cx, &llwa);

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open the state and ft channels 
	somatic_d_channel_open(&daemon_cx, &ft_chan, "llwa_ft", NULL);

	// Get the first force-torque reading and compute the offset with it
	Vector6d ft_data;
	bool gotReading = false;
	while(!gotReading) 
		gotReading = getFT(daemon_cx, ft_chan, ft_data);
	computeOffset(ft_data);
}

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	Vector6d raw, ideal;
	while(!somatic_sig_received) {
		
		c++;

		// Move the arm to any position with the joystick
		setJoystickInput(daemon_cx, js_chan, llwa, llwa);
		somatic_motor_update(&daemon_cx, &llwa);
	
		// Print joint values
		if(c % 1000000 == 0) {
			cout << "\nq: ";
			for(size_t i = 0; i < 7; i++) cout << llwa.pos[i] << ", ";
		}

		// Get the f/t sensor data and compute the ideal value
		bool result = (c % 1000000 == 0) && getFT(daemon_cx, ft_chan, raw);
		if(!result) continue;
		ideal = raw + offset;
		cout << "\nft raw: " << raw.transpose() << " " << llwa.pos[6] <<endl;
		cout << "ft ideal: " << ideal.transpose() << " " << llwa.pos[6] <<endl;
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}
