/**
 * @file sensors.cpp
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The source file for the sensor class and helper functions associated with Krang.
 */

#include "sensors.h"
#include <vector>

using namespace std;
using namespace Eigen;

namespace Krang {

/* ******************************************************************************************** */
FT::FT (GripperType type, somatic_d_t* daemon_cx_, dynamics::SkeletonDynamics* r_, Side side_) {

	// Sanity check the input and set the local bariables
	assert((r_ != NULL) && "Can not initialize the f/t sensors without robot kinematics");
	robot = r_;
	daemon_cx = daemon_cx_;
	side = side_;

	// Open the data channel
	chan = new ach_channel_t();
	somatic_d_channel_open(daemon_cx, chan, (side == LEFT) ? "llwa_ft" : "rlwa_ft", NULL);

	// Set the gripper mass and center of mass based on the type: the com for ext, schunk and robotiq
	// are 0.026, 0.0683, 0.065 (?)
	gripperMass = 0.169;			//< f/t extension mass (will be there regardless of type)
	if(type == GRIPPER_TYPE_ROBOTIQ) {
		gripperMass += 2.3;
		gripperCoM = Vector3d(0.0, 0.0, 0.09);
	}
	else if (type == GRIPPER_TYPE_SCHUNK) {
		gripperMass += 1.6;
		gripperCoM = Vector3d(0.0, -0.008, 0.091);
	}
	else gripperCoM = Vector3d(0.0, -0.008, 0.013);

	// Average the first 1000 readings to compute the offset
	Vector6d data = Vector6d::Zero(), temp;
	for(size_t i = 0; i < 1e3; i++) {
		bool gotReading = false;
		while(!gotReading) gotReading = getRaw(temp);
		data += temp;
	}
	data /= 1e3;

	// Compute the offset between what the reading should be and what it is assuming no external
	// forces
	error(data, offset);	
}

/* ******************************************************************************************** */
void FT::error(const Vector6d& reading, Vector6d& error) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -gripperCoM(2), gripperCoM(1), gripperCoM(2), 
		0.0, -gripperCoM(0), -gripperCoM(1), gripperCoM(0), 0.0;

	// Get the rotation between the world frame and the sensor frame. 
	const char* nodeName = (side == LEFT) ? "lGripper" : "rGripper";
	Matrix3d R = robot->getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();

	// Create the wrench with computed rotation to change the frame from the world to the sensor
	Matrix6d pSsensor_world = MatrixXd::Identity(6,6); 
	pSsensor_world.topLeftCorner<3,3>() = R;
	pSsensor_world.bottomRightCorner<3,3>() = R;
	
	// Get the weight vector (note that we use the world frame for gravity so towards -y)
	Vector6d weightVector_in_world;
	weightVector_in_world << 0.0, 0.0, -gripperMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d expectedFT = pTcom_sensor * pSsensor_world * weightVector_in_world;

	// Compute the difference between the actual and expected f/t values
	error = expectedFT - reading;
}

/* ******************************************************************************************** */
void FT::updateExternal () {

	// Attempt to get the current reading. If failed, return the last value
	Vector6d raw;
	if(!getRaw(raw)) return;

	// Supplement raw with the offset
	Vector6d ideal = raw + offset;

	// Otherwise, compute the external reading as the difference between expected reading due to
	// gripper weight and the current ideal reading
	error(ideal, lastExternal);
}

/* ******************************************************************************************** */
bool FT::getRaw (Vector6d& raw) {

	// Check if there is anything to read
	// TODO Remove aa_tm_future
	int result;
	size_t numBytes = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
	uint8_t* buffer = (uint8_t*) somatic_d_get(daemon_cx, chan, &numBytes, &abstimeout, ACH_O_LAST, 
			&result);

	// Return if there is nothing to read
	if(numBytes == 0) return false;

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx->pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return false;
	if(msg->meta->type != SOMATIC__MSG_TYPE__FORCE_MOMENT) return false;

	// Read the force-torque message and write it into the vector
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(daemon_cx->pballoc), 
		numBytes, buffer);
	for(size_t i = 0; i < 3; i++) raw(i) = ftMessage->force->data[i]; 
	for(size_t i = 0; i < 3; i++) raw(i+3) = ftMessage->moment->data[i]; 
	return true;
}

/* ******************************************************************************************** */
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
			 filter_kalman_t* kf) {

	// ======================================================================
	// Get the readings

    // Get a message
    int r;
    struct timespec timeout = aa_tm_future(aa_tm_sec2timespec(1.0/30.0));
    Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
                                                        &protobuf_c_system_allocator,
                                                        IMU_CHANNEL_SIZE,
                                                        imuChan,
                                                        &timeout);
    assert((imu_msg != NULL) && "Didn't get IMU message!");
	// // Get the value
	// int r;
	// struct timespec currTime;
	// clock_gettime(CLOCK_MONOTONIC, &currTime);
	// struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	// Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
	// 		&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, imuChan, &abstime );
	// assert((imu_msg != NULL) && "Imu message is faulty!");

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	_imu = atan2(newX, imu_msg->data[2]); 
	_imuSpeed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// ======================================================================
	// Filter the readings

	// Skip if a filter is not provided
	if(kf == NULL) return;

	// Setup the data
	kf->z[0] = _imu, kf->z[1] = _imuSpeed;

	// Setup the time-dependent process matrix
	kf->A[0] = kf->A[3] = 1.0;
	kf->A[2] = dt;

	// Setup the process noise matrix
	static const double k1 = 2.0;
	static const double k1b = 5.0;
	kf->R[0] = (dt*dt*dt*dt) * k1 * (1.0 / 4.0);
	kf->R[1] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[2] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[3] = (dt*dt) * k1b;
	
	// First make a prediction of what the reading should have been, then correct it
	filter_kalman_predict(kf);
	filter_kalman_correct(kf);

	// Set the values
	_imu = kf->x[0], _imuSpeed = kf->x[1];
}

};	// end of namespace
