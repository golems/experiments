/**
 * @file helpers.cpp
 * @author Can Erdogan
 * @date June 12, 2013
 * @brief This file contains some helper functions such as reading force/torque data if available.
 */

#include "helpers.h"

using namespace std;
using namespace dynamics;
using namespace kinematics;

vector <int> imuWaist_ids; 	///< The index vector to set config of waist/imu 

/* ******************************************************************************************** */
void computeExternal (double imu, double waist, const somatic_motor_t& lwa, const Vector6d& 
		input, SkeletonDynamics& robot, Vector6d& external, bool left) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame by setting the arm values
	// and the imu/waist values
	robot.setConfig(left ? left_arm_ids : right_arm_ids, Map <Vector7d> (lwa.pos));
	robot.setConfig(imuWaist_ids, Vector2d(imu, waist));
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d Rsw = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();
	
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
	external = pSsensor_world.transpose() * external;	
}

/* ******************************************************************************************** */
void computeOffset (double imu, double waist, const somatic_motor_t& lwa, const Vector6d& raw, 
		SkeletonDynamics& robot, Vector6d& offset, bool left) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame. 
	robot.setConfig(imuWaist_ids, Vector2d(imu, waist));
	robot.setConfig(left ? left_arm_ids : right_arm_ids, Map <Vector7d> (lwa.pos));
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d R = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();

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
void initWholeArm (somatic_d_t& daemon_cx, somatic_motor_t& lwa, ach_channel_t& ft_chan, 
		Vector6d& offset, ach_channel_t& imuChan, ach_channel_t& waistChan, bool left) {

	// =======================================================================
	// Initialize the ft and arm channels

	// Sanity check that the world is setup
	assert(world != NULL && "Before initializing an arm the world should be loaded");

	// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
	if(left) system("netcanftd -v -d -I lft -b 1 -B 1000 -c llwa_ft -k -r");
	else system("netcanftd -v -d -I rft -b 9 -B 1000 -c rlwa_ft -k -r");

	// Initialize the arm motors
	if(left) initArm(daemon_cx, lwa, "llwa");
	else initArm(daemon_cx, lwa, "rlwa"); 
	somatic_motor_update(&daemon_cx, &lwa);

	// Open the ft channel
	if(left) somatic_d_channel_open(&daemon_cx, &ft_chan, "llwa_ft", NULL);
	else somatic_d_channel_open(&daemon_cx, &ft_chan, "rlwa_ft", NULL);

	// =======================================================================
	// Compute the offset between the raw ft readings and the truth

	// Get the first force-torque reading and compute the offset with it
	cout << "reading FT now" << endl;
	Vector6d ft_data, temp;
	for(size_t i = 0; i < 1e3; i++) {
		while(!getFT(daemon_cx, ft_chan, temp));
		ft_data += temp;
	}
	ft_data /= 1e3;

	// Get imu data
	double imu = 0.0;
	for(int i = 0; i < 500; i++) {
		double temp;
		getImu(&temp, imuChan);
		imu += temp;
	}
	imu /= 500;
	cout << "imu : " << imu*180.0/M_PI << endl;

	// Get waist data
	double waist;
	while(!getWaist(&waist, waistChan));
	cout << "waist : " << waist*180.0/M_PI << endl;

	// Using imu, waist and ft data, compute the offset
	computeOffset(imu, waist, lwa, ft_data, *(world->getSkeleton(0)), offset, left);
}

/* ********************************************************************************************* */
void init (somatic_d_t& daemon_cx, ach_channel_t& js_chan, ach_channel_t& imuChan, 
		ach_channel_t& waistChan, Arm* left, Arm* right) {

	// Kill any netcanftds around
	system("killall -s 9 netcanftd");
	usleep(20000);

	// Set up the index vectors for the imu and waist angles
	imuWaist_ids.push_back(5);	
	imuWaist_ids.push_back(8);	

	// Set up the arm index vectors
	int left_arm_ids_a [] = {11, 13, 15, 17, 19, 21, 23};
	int right_arm_ids_a [] = 	{12, 14, 16, 18, 20, 22, 24};
	for(size_t i = 0; i < 7; i++) left_arm_ids.push_back(left_arm_ids_a[i]);
	for(size_t i = 0; i < 7; i++) right_arm_ids.push_back(right_arm_ids_a[i]);

	// Load environment from dart for kinematics
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "04-compliance";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the channels to the imu and waist sensors, and to joystick channel
	somatic_d_channel_open(&daemon_cx, &js_chan, "joystick-data", NULL);
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);

	// Make calls for the arms if necessary
	if(left != NULL) initWholeArm(daemon_cx, left->lwa, left->ft_chan, left->offset, 
		imuChan, waistChan, true);
	if(right != NULL) initWholeArm(daemon_cx, right->lwa, right->ft_chan, right->offset,
		imuChan, waistChan, false);
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
bool getWaist(double* waist, ach_channel_t& waistChan) {
	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);

	// Get the data from the motors
	int r;
	Somatic__MotorState * waistState = SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, 
		&protobuf_c_system_allocator, 1024, &waistChan, &abstime);
	
	// Ach sanity check
	aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME,
			"Ach wait failure %s on pcio data receive (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)),
			__FILE__, __LINE__);

	if (r == ACH_OK) {
		// Read the data
		*waist = waistState->position->data[0];
		
		// Free the memory
		somatic__motor_state__free_unpacked(waistState, &protobuf_c_system_allocator );
		return true;
	}
	return false;
}

/* ********************************************************************************************* */
void getImu (double *imu, ach_channel_t& imuChan) {

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Prepare the ssdmu structure 
	ssdmu_sample_t imu_sample;
	imu_sample.x  = imu_msg->data[0];
	imu_sample.y  = imu_msg->data[1];
	imu_sample.z  = imu_msg->data[2];
	imu_sample.dP = imu_msg->data[3];
	imu_sample.dQ = imu_msg->data[4];
	imu_sample.dR = imu_msg->data[5];

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// Make the calls to extract the pitch and rate of extraction
	//*imu = -ssdmu_pitch(&imu_sample) + M_PI/2;				 
}
/* ********************************************************************************************* */

