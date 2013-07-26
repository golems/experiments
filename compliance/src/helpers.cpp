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

vector <int> arm_ids;		///< The index vector to set config of arms
vector <int> imuWaist_ids; ///< The index vector to set config of waist/imu 

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
	robot.setConfig(arm_ids, Map <Vector7d> (lwa.pos));
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
	robot.setConfig(arm_ids, Map <Vector7d> (lwa.pos));
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d R = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();
	cout << "Transform : "<< endl << R << endl;
	vector <int> dofs;
	for(size_t i = 0; i < 25; i++) dofs.push_back(i);
	cout << "\nq in computeExternal: " << robot.getConfig(dofs).transpose() << endl;
	
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
void init (somatic_d_t& daemon_cx, ach_channel_t& js_chan, ach_channel_t& imuChan, 
		ach_channel_t& waistChan, ach_channel_t& ft_chan, somatic_motor_t& lwa, Vector6d& offset, 
		bool left){

	// Set up the index vectors
	int left_arm_ids_a [] = {11, 13, 15, 17, 19, 21, 23};
	int right_arm_ids_a [] = 	{12, 14, 16, 18, 20, 22, 24};
	int * arm_ids_a = left ? left_arm_ids_a : right_arm_ids_a;
	for(size_t i = 0; i < 7; i++) arm_ids.push_back(arm_ids_a[i]);
	imuWaist_ids.push_back(5);	
	imuWaist_ids.push_back(8);	

	// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
	
	if(left) { 
		system("sns -k lft"); 
                usleep(20000);
		system("netcanftd -v -d -I lft -b 1 -B 1000 -c llwa_ft -k -r");
	}	else { 
		system("sns -k rft");
                usleep(20000);
		system("netcanftd -v -d -I rft -b 9 -B 1000 -c rlwa_ft -k -r");
	}

	// Load environment from dart for kinematics
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "saul-01-gripperWeight";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the left arm
	if(left) initArm(daemon_cx, lwa, "llwa");
	else{ initArm(daemon_cx, lwa, "rlwa"); }
	somatic_motor_update(&daemon_cx, &lwa);
	usleep(1e5);

	// Initialize the channels to the imu and waist sensors
	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	usleep(1e5);

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

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Open the state and ft channels
	if(left) somatic_d_channel_open(&daemon_cx, &ft_chan, "llwa_ft", NULL);
	else somatic_d_channel_open(&daemon_cx, &ft_chan, "rlwa_ft", NULL);

	// Get the first force-torque reading and compute the offset with it
	cout << "reading FT now" << endl;
	Vector6d ft_data, temp;
	ft_data << 0,0,0,0,0,0;
	for(size_t i = 0; i < 1e3; i++) {
		bool gotReading = false;
		while(!gotReading) 
			gotReading = getFT(daemon_cx, ft_chan, temp);
		ft_data += temp;
	}
	ft_data /= 1e3;
	
	computeOffset(imu, waist, lwa, ft_data, *(world->getSkeleton(0)), offset, left);
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
// Reads waist data and returns true if data successfully updated
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

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	*imu = -atan2(newX, imu_msg->data[2]) + M_PI/2;

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );
}

/* ********************************************************************************************* */

