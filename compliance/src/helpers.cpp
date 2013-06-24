/**
 * @file helpers.cpp
 * @author Can Erdogan
 * @date June 12, 2013
 * @brief This file contains some helper functions such as reading force/torque data if available.
 */

#include "helpers.h"


using namespace kinematics;
using namespace dynamics;
using namespace std;

std::vector <int> arm_ids;		///< The index vector to set config of arms

/* ******************************************************************************************** */
void computeExternal (const somatic_motor_t& llwa, const Vector6d& input, SkeletonDynamics& robot, 
		Vector6d& external) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame. 
	robot.setConfig(arm_ids, Map <Vector7d> (llwa.pos));
	Matrix3d Rsw = robot.getNode("lGripper")->getWorldTransform().topLeftCorner<3,3>().transpose();
	
	// Create the wrench with computed rotation to change the frame from the world to the sensor
	Matrix6d pSsensor_world = MatrixXd::Identity(6,6); 
	pSsensor_world.topLeftCorner<3,3>() = Rsw;
	pSsensor_world.bottomRightCorner<3,3>() = Rsw;
	
	// Get the weight vector (note that we use the world frame for gravity so towards -y)
	// static const double eeMass = 0.169;	// kg - ft extension
	Vector6d weightVector_in_world;
	weightVector_in_world << 0.0, 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d wrenchWeight = pTcom_sensor * pSsensor_world * weightVector_in_world;

	// Remove the effect from the sensor value and convert the wrench into the world frame
	external = input - wrenchWeight;
	external = pSsensor_world.transpose() * external;	// Comment this line out for 03 experiments!!!
}

/* ******************************************************************************************** */
void forwardKinematics (const somatic_motor_t& arm, MatrixXd& Tbee) {

	// Create the DH table for the arm
	double T [7][4] = {
		{0.0, -M_PI_2, -L4, arm.pos[0]},
		{0.0, M_PI_2, 0.0, arm.pos[1]},
		{0.0, -M_PI_2, -L5, arm.pos[2]},
		{0.0, M_PI_2, 0.0, arm.pos[3]},
		{0.0, -M_PI_2, -L6, arm.pos[4]},
		{0.0, M_PI_2, 0.0, arm.pos[5]},
		{0.0, 0.0, -L7-L8, arm.pos[6]}};

	// Loop through the joints and aggregate the transformations multiplying from left
	Tbee = MatrixXd::Identity(4,4);
	for(size_t i = 0; i < 7; i++) 
		Tbee *= dh(T[i][0], T[i][1], T[i][2], T[i][3]);		
}

/* ******************************************************************************************** */
void computeOffset (const somatic_motor_t& arm, const Vector6d& raw, SkeletonDynamics& robot, 
		Vector6d& offset) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame. 
	robot.setConfig(arm_ids, Map <Vector7d> (arm.pos));
	Matrix3d R = robot.getNode("lGripper")->getWorldTransform().topLeftCorner<3,3>().transpose();

	// Create the wrench with computed rotation to change the frame from the bracket to the sensor
	Matrix6d pSsensor_bracket = MatrixXd::Identity(6,6); 
	pSsensor_bracket.topLeftCorner<3,3>() = R;
	pSsensor_bracket.bottomRightCorner<3,3>() = R;
	
	// Get the weight vector (note that we use the bracket frame for gravity so towards -y)
	Vector6d weightVector_in_bracket;
	weightVector_in_bracket << 0.0, 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d expectedFT = pTcom_sensor * pSsensor_bracket * weightVector_in_bracket;
	pv(raw);
	pv(expectedFT);

	// Compute the difference between the actual and expected f/t values
	offset = expectedFT - raw;
	pv(offset);
}

/* ********************************************************************************************* */
void init (somatic_d_t& daemon_cx, ach_channel_t& js_chan, ach_channel_t& ft_chan, 
		somatic_motor_t& llwa, Vector6d& offset){

	int arm_ids_a [] = {10, 12, 14, 16, 18, 20, 22};
	for(size_t i = 0; i < 7; i++) arm_ids.push_back(arm_ids_a[i]);

	/// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
	system("killall -s 9 netcanftd");
	usleep(20000);
	system("netcanftd -v -d -I lft -b 2 -B 1000 -c llwa_ft -k -r");

	DartLoader dl;
	simulation::World* mWorld = dl.parseWorld("../scenes/01-World-Robot.urdf");
	assert((mWorld != NULL) && "Could not find the world");

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
	Vector6d ft_data, temp;
	ft_data << 0,0,0,0,0,0;
	for(size_t i = 0; i < 1e3; i++) {
		bool gotReading = false;
		while(!gotReading) 
			gotReading = getFT(daemon_cx, ft_chan, temp);
		ft_data += temp;
	}
	ft_data /= 1e3;
	computeOffset(llwa, ft_data, *(mWorld->getSkeleton(0)), offset);
}

/* ********************************************************************************************* */
bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data) {

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &ft_chan, &numBytes, &abstimeout, 
		ACH_O_LAST, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return false;

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return false;
	if(msg->meta->type != SOMATIC__MSG_TYPE__FORCE_MOMENT) return false;

	// Read the force-torque message and write it into the vector
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(daemon_cx.pballoc), 
		numBytes, buffer);
	for(size_t i = 0; i < 3; i++) data(i) = ftMessage->force->data[i]; 
	for(size_t i = 0; i < 3; i++) data(i+3) = ftMessage->moment->data[i]; 
	return true;
}
/* ********************************************************************************************* */
