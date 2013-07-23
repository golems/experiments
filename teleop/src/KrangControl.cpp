/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** \file KrangControl.cpp
 *
 *  \author Jonathan Scholz
 *  \date 7/17/2013
 */

#include <unistd.h>
#include <imud.h>
#include "KrangControl.h"

#include <kinematics/BodyNode.h> //FIXME

/*################################################################################################
  #
  ################################################################################################*/
KrangControl::KrangControl() {}

KrangControl::~KrangControl() {
	// TODO Auto-generated destructor stub
}

/*################################################################################################
  # PUBLIC INITIALIZATION METHODS
  ################################################################################################*/
int KrangControl::initialize(simulation::World* world, somatic_d_t *daemon_cx, const char *robot_name) {
    this->send_motor_cmds = false;
    this->initialized = false;
    this->_current_mode = false;
    
	this->_world = world;
	this->_krang = _world->getSkeleton(robot_name);
	this->_daemon_cx = daemon_cx;
	this->_arm_motors.resize(2);
	this->_arm_ids.resize(2);
	this->_gripper_nodes.resize(2);
	this->_schunk_gripper_motors.resize(2);
	this->_robotiq_gripper_channels.resize(2);
	setDartIDs();
	return 0;
}

void KrangControl::initSomatic() {
	if (!initialized) {
		initialized = true;
		initArm(LEFT_ARM, "llwa");
		initArm(RIGHT_ARM, "rlwa");
		//initSchunkGripper(LEFT_ARM, "lGripper");
		//initSchunkGripper(RIGHT_ARM, "rGripper");
		initRobotiqGripper(LEFT_ARM, "lgripper-cmd");
		initRobotiqGripper(RIGHT_ARM, "rgripper-cmd");
		initWaist();
		initTorso();
		initIMU();
		initFT();

#ifdef EXPERIMENTAL
		initPIDs(LEFT_ARM);
		initPIDs(RIGHT_ARM);
#endif
	}
}

void KrangControl::setMotorOutputMode(bool mode) {
	this->send_motor_cmds = mode;

	if (send_motor_cmds) {
		initSomatic();
		resetMotors();
	}
	else
		halt();

}

/*################################################################################################
  # ROBOT UPDATE METHODS
  ################################################################################################*/
/*
 * Reads the full Krang configuration from somatinc and updates the krang skeleton in the
 * provided world.
 */
bool KrangControl::updateKrangSkeleton() {
	if (!initialized)
		return false;

	if (!updateRobotSkelFromSomaticMotor(_arm_motors[LEFT_ARM], _arm_ids[LEFT_ARM])) return false;
	if (!updateRobotSkelFromSomaticMotor(_arm_motors[RIGHT_ARM], _arm_ids[RIGHT_ARM])) return false;
	updateRobotSkelFromSomaticMotor(_torso, _torso_ids);
	updateRobotSkelFromSomaticWaist();
	updateRobotSkelFromIMU();
	return true;
}

void KrangControl::setArmConfig(lwa_arm_t arm, Eigen::VectorXd &config) {
	_krang->setConfig(_arm_ids[arm], config);
}

/*
 * The force sensor is prone to drift over time, so we use this function to
 * attempt to estimate the zero-point of the sensor.
 * Currently we are taking a simple average of a bunch of readings, which
 * obviously assumes that there are no external forces acting on the sensor.
 */
void KrangControl::updateFTOffset(lwa_arm_t arm) {
	if (!initialized)
		return;

	// Get a nice, clean force-torque reading
	Eigen::VectorXd ft_raw_initial_reading(6);
	ft_raw_initial_reading << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	for(int i = 0; i < _ft_init_iters; i++)
		ft_raw_initial_reading += getFT(arm, true);

	ft_raw_initial_reading /= (double)_ft_init_iters;

	// Get the point transform wrench due to moving the affected
	// position from com to sensor origin. The transform is an
	// identity with the bottom left a skew symmetric of the point
	// translation
	Eigen::MatrixXd transform_eecom_sensor = Eigen::MatrixXd::Identity(6,6);
	transform_eecom_sensor.bottomLeftCorner<3,3>() <<
			0.0, -_robotiq_com(2), _robotiq_com(1),
			_robotiq_com(2), 0.0, -_robotiq_com(0),
			-_robotiq_com(1), _robotiq_com(0), 0.0;

	// figure out the rotation of our end effector, just the rotation
	Eigen::Matrix3d ee_rotation = getEffectorPose(arm).topLeftCorner<3,3>().transpose();

	// Create the wrench with computed rotation to change the frame
	// from the world to the sensor.
	Eigen::MatrixXd wrenchrotate_sensor_world = Eigen::MatrixXd::Identity(6,6);
	wrenchrotate_sensor_world.topLeftCorner<3,3>() = ee_rotation;
	wrenchrotate_sensor_world.bottomRightCorner<3,3>() = ee_rotation;

	// Get the weight vector (note that we use the world frame for
	// gravity so towards -z)
	Eigen::VectorXd end_effector_weight_in_world(6);
	end_effector_weight_in_world << 0.0, 0.0, _robotiq_mass * -9.81, 0.0, 0.0, 0.0;

	// Compute what the force and torque should be without any external values by multiplying the
	// position and rotation transforms with the expected effect of the gravity
	Eigen::VectorXd expected_ft_reading = transform_eecom_sensor * wrenchrotate_sensor_world * end_effector_weight_in_world;

	// Compute the difference between the actual and expected f/t
	// values and return the result
	Eigen::VectorXd offset = expected_ft_reading - ft_raw_initial_reading;

	// And store it
	_ft_offsets[arm] = offset;
}

/*################################################################################################
  # PUBLIC ROBOT QUERY METHODS
  ################################################################################################*/
Eigen::VectorXd KrangControl::getArmConfig(lwa_arm_t arm) {
	return _krang->getConfig(_arm_ids[arm]);
}


Eigen::Matrix4d KrangControl::getEffectorPose(lwa_arm_t arm) {
	return _gripper_nodes[arm]->getWorldTransform();
}

Eigen::MatrixXd KrangControl::getEffectorJacobian(lwa_arm_t arm) {
	// Get the Jacobian of the indicated gripper, for computing joint-space velocities
	Eigen::MatrixXd Jlin = _gripper_nodes[arm]->getJacobianLinear().topRightCorner<3,7>();
	Eigen::MatrixXd Jang = _gripper_nodes[arm]->getJacobianAngular().topRightCorner<3,7>();
	Eigen::MatrixXd J (6,7);
	J << Jlin, Jang;
	return J;
}

/*
 * Returns the current force-torque sensor reading for the given arm
 * in the world frame, with the gripper modeled off
 */
Eigen::VectorXd KrangControl::getFtWorldWrench(lwa_arm_t arm) {
	Eigen::VectorXd external(6);
	if (!initialized)
		return external;

	// get a measurement from the sensor
	Eigen::VectorXd raw_ft_reading = getFT(arm);

	// use the offset to correct it
	Eigen::VectorXd corrected_ft_reading = raw_ft_reading + _ft_offsets[arm];

	// Get the point transform wrench due to moving the affected
	// position from com to sensor origin. The transform is an
	// identity with the bottom left a skew symmetric of the point
	// translation
	Eigen::MatrixXd transform_eecom_sensor = Eigen::MatrixXd::Identity(6,6);
	transform_eecom_sensor.bottomLeftCorner<3,3>() <<
			0.0, -_robotiq_com(2), _robotiq_com(1),
			_robotiq_com(2), 0.0, -_robotiq_com(0),
			-_robotiq_com(1), _robotiq_com(0), 0.0;

	// figure out how our end effector is rotated by giving dart the
	// arm values and the imu/waist values
	Eigen::Matrix3d ee_rotation = getEffectorPose(arm).topLeftCorner<3,3>().transpose();

	// Create the wrench with computed rotation to change the frame
	// from the world to the sensor
	Eigen::MatrixXd wrenchrotate_sensor_world = Eigen::MatrixXd::Identity(6,6);
	wrenchrotate_sensor_world.topLeftCorner<3,3>() = ee_rotation;
	wrenchrotate_sensor_world.bottomRightCorner<3,3>() = ee_rotation;

	// Get the weight vector (note that we use the world frame for
	// gravity so towards -z)
	Eigen::VectorXd end_effector_weight_in_world(6);
	end_effector_weight_in_world << 0.0, 0.0, _robotiq_mass * -9.81, 0.0, 0.0, 0.0;

	// Compute what the force and torque should be without any
	// external values by multiplying the position and rotation
	// transforms with the expected effect of the gravity
	Eigen::VectorXd expected_ft_reading = transform_eecom_sensor * wrenchrotate_sensor_world * end_effector_weight_in_world;

	// Remove the effect from the sensor value, convert the wrench
	// into the world frame, and return the result
	external = corrected_ft_reading - expected_ft_reading;
	external = wrenchrotate_sensor_world.transpose() * external;
	return external;
}

/*################################################################################################
  # PUBLIC ROBOT CONTROL METHODS
  ################################################################################################*/


void KrangControl::setRobotArmVelocities(lwa_arm_t arm, Eigen::VectorXd& qdot, double dt) {

	if (send_motor_cmds) {
		sendRobotArmVelocities(arm, qdot, dt);
	}
	else
		fakeArmMovement(arm, qdot, dt);
}

// super basic open and close actions, for now
void KrangControl::setRobotiqGripperAction(lwa_arm_t arm, const Eigen::VectorXi& buttons) {

	robotiqd_achcommand_t rqd_msg;
	rqd_msg.mode = GRASP_BASIC;
	rqd_msg.grasping_speed = 0xff;
	rqd_msg.grasping_force = 0xff;

	if (buttons[0]) rqd_msg.grasping_pos = 0x00;
	if (buttons[1]) rqd_msg.grasping_pos = 0xff;

	if (buttons[0] || buttons[1])
		ach_put(&_robotiq_gripper_channels[arm], &rqd_msg, sizeof(rqd_msg));
}

void KrangControl::halt() {

	somatic_motor_halt(_daemon_cx, &_arm_motors[LEFT_ARM]);
	somatic_motor_halt(_daemon_cx, &_arm_motors[RIGHT_ARM]);
	if (send_motor_cmds != false)
		send_motor_cmds = false;
}

void KrangControl::resetMotors()
{
	somatic_motor_reset(_daemon_cx, &_arm_motors[LEFT_ARM]);
	somatic_motor_reset(_daemon_cx, &_arm_motors[RIGHT_ARM]);
}

/*################################################################################################
  # PROTECTED CONTROL HELPERS
  ################################################################################################*/
void KrangControl::sendRobotArmVelocities(lwa_arm_t arm, Eigen::VectorXd& qdot, double dt) {
	if (!initialized || !send_motor_cmds)
		return;

#ifdef EXPERIMENTAL
	if (this->_current_mode) {
		setPIDQdotRef(arm, qdot);
		Eigen::VectorXd qRef = getPIDQref(arm);
		qRef += qdot*dt;
		setPIDQRef(arm, qRef);
		Eigen::VectorXd currents = updatePIDs(arm);
//		somatic_motor_cmd(_daemon_cx, &_arm_motors[arm], SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, currents.data(), 7, NULL);
	} else
#endif
		somatic_motor_cmd(_daemon_cx, &_arm_motors[arm], SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, qdot.data(), 7, NULL);
}

void KrangControl::fakeArmMovement(lwa_arm_t arm, Eigen::VectorXd& qdot, double dt) {

	Eigen::VectorXd q = _krang->getConfig(_arm_ids[arm]);
	q += qdot * dt;
	_krang->setConfig(_arm_ids[arm], q);
}


/*################################################################################################
  # PROTECTED INITIALIZATION METHODS
  ################################################################################################*/
void KrangControl::setDartIDs() {
	// IDs for various krang body parts, hard-coded for now

	int idL[] = {11, 13, 15, 17, 19, 21, 23};
	_arm_ids[LEFT_ARM] = std::vector<int>(idL, idL + sizeof(idL)/sizeof(idL[0]));

	int idR[] = {12, 14, 16, 18, 20, 22, 24};
	_arm_ids[RIGHT_ARM] = std::vector<int>(idR, idR + sizeof(idR)/sizeof(idR[0]));

	int imu_ids[] = {5};
	_imu_ids = std::vector<int>(imu_ids, imu_ids + sizeof(imu_ids)/sizeof(imu_ids[0]));

	int waist_ids[] = {8};
	_waist_ids = std::vector<int>(waist_ids, waist_ids + sizeof(waist_ids)/sizeof(waist_ids[0]));

	int torso_ids[] = {9};
	_torso_ids = std::vector<int>(torso_ids, torso_ids + sizeof(torso_ids)/sizeof(torso_ids[0]));

	//TODO factor out to separate func for cleanness
	_gripper_nodes[LEFT_ARM] = this->_krang->getNode("lGripper");
	_gripper_nodes[RIGHT_ARM] = this->_krang->getNode("rGripper");
}

void KrangControl::initWaist() {

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(_daemon_cx, &_waist, 2, "waist-cmd", "waist-state");
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
			&_waist.pos_valid_min, &_waist.vel_valid_min,
			&_waist.pos_limit_min, &_waist.pos_limit_min,
			&_waist.pos_valid_max, &_waist.vel_valid_max,
			&_waist.pos_limit_max, &_waist.pos_limit_max};
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 2);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 2);

	// Update and reset them
	somatic_motor_update(_daemon_cx, &_waist);
}

void KrangControl::initTorso() {

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(_daemon_cx, &_torso, 1, "torso-cmd", "torso-state");
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
			&_torso.pos_valid_min, &_torso.vel_valid_min,
			&_torso.pos_limit_min, &_torso.pos_limit_min,
			&_torso.pos_valid_max, &_torso.vel_valid_max,
			&_torso.pos_limit_max, &_torso.pos_limit_max};
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 1);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 1);

	// Update and reset them
	somatic_motor_update(_daemon_cx, &_torso);
}

void KrangControl::initIMU() {

	somatic_d_channel_open(_daemon_cx, &_imu_chan, "imu-data", NULL);

	// Set the offset values to amc motor group so initial wheel pos readings are zero
	getIMU();

	usleep(1e5);

	// Initialize kalman filter for the imu and set the measurement and meas. noise matrices
	// Also, set the initial reading to the current imu reading to stop moving from 0 to current
	_imu_kf = new filter_kalman_t;
	filter_kalman_init(_imu_kf, 2, 0, 2);
	_imu_kf->C[0] = _imu_kf->C[3] = 1.0;
	_imu_kf->Q[0] = _imu_kf->Q[3] = 1e-3;
	_imu_kf->x[0] = _imu_angle, _imu_kf->x[1] = _imu_speed;
}

void KrangControl::initFT() {
	_ft_offsets.resize(2);
	_ft_channels.resize(2);
	_robotiq_com << 0.0, -0.008, 0.091; ///< COM: 0.065 for robotiq itself, 0.026 length of ext + 2nd
	somatic_d_channel_open(_daemon_cx, &_ft_channels[LEFT_ARM], "llwa_ft", NULL);
	somatic_d_channel_open(_daemon_cx, &_ft_channels[RIGHT_ARM], "rlwa_ft", NULL);
	updateFTOffset(LEFT_ARM);
	updateFTOffset(RIGHT_ARM);
}

void KrangControl::initArm(lwa_arm_t arm, const char* armName) {
std::cout << "initializing arm " << std::endl;
	// Get the channel names
	char cmd_name [16], state_name [16];
	sprintf(cmd_name, "%s-cmd", armName);
	sprintf(state_name, "%s-state", armName);

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(_daemon_cx, &_arm_motors[arm], 7, cmd_name, state_name);
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
			&_arm_motors[arm].pos_valid_min, &_arm_motors[arm].vel_valid_min,
			&_arm_motors[arm].pos_limit_min, &_arm_motors[arm].pos_limit_min,
			&_arm_motors[arm].pos_valid_max, &_arm_motors[arm].vel_valid_max,
			&_arm_motors[arm].pos_limit_max, &_arm_motors[arm].pos_limit_max};
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);

	// Update and reset them
	somatic_motor_update(_daemon_cx, &_arm_motors[arm]);
	somatic_motor_cmd(_daemon_cx, &_arm_motors[arm], SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
	usleep(1e5);
}

void KrangControl::initSchunkGripper(lwa_arm_t gripper, const char* name) {

	// Get the channel names
	char cmd_name [16], state_name [16];
	sprintf(cmd_name, "%s-cmd", name);
	sprintf(state_name, "%s-state", name);

	// Create the motor reference for the left gripper
	somatic_motor_init(_daemon_cx, &_schunk_gripper_motors[gripper], 1, cmd_name, state_name);
	usleep(1e5);

	// Set the min/max values for valid and limits values
	aa_fset(_schunk_gripper_motors[gripper].pos_valid_min, 0.009, 1);
	aa_fset(_schunk_gripper_motors[gripper].pos_limit_min, 0.009, 1);
	aa_fset(_schunk_gripper_motors[gripper].pos_valid_max, 0.068, 1);
	aa_fset(_schunk_gripper_motors[gripper].pos_limit_max, 0.068, 1);
	aa_fset(_schunk_gripper_motors[gripper].vel_valid_min, -0.008, 1);
	aa_fset(_schunk_gripper_motors[gripper].vel_limit_min, -0.008, 1);
	aa_fset(_schunk_gripper_motors[gripper].vel_valid_max, 0.008, 1);
	aa_fset(_schunk_gripper_motors[gripper].vel_limit_max, 0.008, 1);

	// Update and reset them
	somatic_motor_update(_daemon_cx, &_schunk_gripper_motors[gripper]);
//	somatic_motor_cmd(_daemon_cx, &_schunk_gripper_motors[gripper], SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
	usleep(1e5);
}

void KrangControl::initRobotiqGripper(lwa_arm_t arm, const char *chan) {
	// for robotiq we currently talk over ach directly
	ach_open(&_robotiq_gripper_channels[arm], chan, NULL);
}

/*################################################################################################
  # UPDATE METHOD HELPERS
  ################################################################################################*/
bool KrangControl::updateRobotSkelFromSomaticMotor(somatic_motor_t& mot, std::vector<int>& IDs) {
	if (!initialized)
		return false;

	assert(mot.n == IDs.size());
	Eigen::VectorXd vals(mot.n);
	somatic_motor_update(_daemon_cx, &mot);
	for(size_t i = 0; i < mot.n; i++) {
		vals(i) = mot.pos[i];

		if (fabs(mot.cur[i]) > 11.0) {
			std::cout << "over current for motor " << i  << " (" << mot.cur[i] << "). halting." << std::endl;
			halt();
			return false;
		}
	}

	_krang->setConfig(IDs, vals);
	return true;
}

void KrangControl::updateRobotSkelFromSomaticWaist() {
	if (!initialized)
		return;

	// Read the waist's state and update the averaged waist position
	somatic_motor_update(_daemon_cx, &_waist);
	double waist_angle = (_waist.pos[0] - _waist.pos[1]) / 2.0;

	Eigen::VectorXd vals(1);
	vals << waist_angle;
	_krang->setConfig(_waist_ids, vals);
}

void KrangControl::updateRobotSkelFromIMU() {
	if (!initialized)
		return;

	Eigen::VectorXd imu_pos(1);
	getIMU();
	imu_pos << -_imu_angle + M_PI/2;
	_krang->setConfig(_imu_ids, imu_pos);
}

/// Computes the imu value from the imu readings
void KrangControl::getIMU() {
	static bool imu_got_reading = false;
	if (!imu_got_reading) {
		_imu_angle = 0.0;
		_imu_speed = 0.0;
	}

	if (!initialized)
		return;

	// ======================================================================
	// Get the readings

	// Compute timestep
	static double last_movement_time = aa_tm_timespec2sec(aa_tm_now());
	double current_time = aa_tm_timespec2sec(aa_tm_now());
	double dt = current_time - last_movement_time;
	last_movement_time = current_time;

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector,
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &_imu_chan, &abstime);
	if (imu_msg == NULL) return; // _imu_angle and _imu_speed will remain at their last values
	imu_got_reading = true;

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	_imu_angle = atan2(newX, imu_msg->data[2]);
	_imu_speed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// ======================================================================
	// Filter the readings

	// Skip if a filter is not provided
	if(_imu_kf == NULL) return;

	// Setup the data
	_imu_kf->z[0] = _imu_angle, _imu_kf->z[1] = _imu_speed;

	// Setup the time-dependent process matrix
	_imu_kf->A[0] = _imu_kf->A[3] = 1.0;
	_imu_kf->A[2] = dt;

	// Setup the process noise matrix
	static const double k1 = 2.0;
	static const double k1b = 5.0;
	_imu_kf->R[0] = (dt*dt*dt*dt) * k1 * (1.0 / 4.0);
	_imu_kf->R[1] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	_imu_kf->R[2] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	_imu_kf->R[3] = (dt*dt) * k1b;

	// First make a prediction of what the reading should have been, then correct it
	filter_kalman_predict(_imu_kf);
	filter_kalman_correct(_imu_kf);

	// Set the values
	_imu_angle = _imu_kf->x[0], _imu_speed = _imu_kf->x[1];
}

/*################################################################################################
  # FORCE-TORQUE HELPERS
  ################################################################################################*/
Eigen::VectorXd KrangControl::getFT(lwa_arm_t arm, bool wait) {
	static Eigen::VectorXd ft_reading(6);
	static bool ft_got_reading = false;
	if (!ft_got_reading) ft_reading.setZero();

	if (!initialized)
		return ft_reading;

	// get a message
	int r;
	size_t num_bytes = 0;
	Somatic__ForceMoment* ft_msg;
	if (wait) {
		struct timespec timeout = aa_tm_future(aa_tm_sec2timespec(1.0/100.0));
		ft_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__force_moment,
				&protobuf_c_system_allocator,
				1024,
				&_ft_channels[arm],
				&timeout);
	}
	else {
		ft_msg = SOMATIC_GET_LAST_UNPACK(r, somatic__force_moment,
				&protobuf_c_system_allocator,
				1024,
				&_ft_channels[arm]);
	}
	if (ft_msg == NULL) {
		return ft_reading;
	}

	ft_got_reading = true;

	// extract the data into something we can use
	for(size_t i = 0; i < 3; i++) ft_reading(i) = ft_msg->force->data[i];
	for(size_t i = 0; i < 3; i++) ft_reading(i+3) = ft_msg->moment->data[i];

	// free the unpacked message
	somatic__force_moment__free_unpacked(ft_msg, &protobuf_c_system_allocator);

	// and return our result
	return ft_reading;
}

#ifdef EXPERIMENTAL
/*################################################################################################
  # EXPERIMENTAL
 ################################################################################################*/

Eigen::VectorXd KrangControl::updatePIDs(lwa_arm_t arm) {
	if (!initialized)
		return ft_reading;

	Eigen::VectorXd result(7);

	double p_p_value;
    double p_d_value;
    double v_p_value;
    double v_d_value;

    double pos_error;
    double vel_error;

    for(int i = 0; i < _arm_motors[arm].n; i++) {
        result[i] = 0;

        if(_pids[arm][i].use_pos) {
            pos_error = _pids[arm][i].pos_target - _arm_motors[arm].pos[i];

            p_p_value = _pids[arm][i].K_p_p * pos_error;
            p_d_value = _pids[arm][i].K_p_d * (pos_error - _pids[arm][i].pos_error_last);

            result[i] += p_p_value + p_d_value;

            _pids[arm][i].pos_error_last = pos_error;
        }
        if (_pids[arm][i].use_vel) {
            vel_error = _pids[arm][i].vel_target - _arm_motors[arm].vel[i];

            v_p_value = _pids[arm][i].K_v_p * vel_error;
            v_d_value = _pids[arm][i].K_v_d * (vel_error - _pids[arm][i].vel_error_last);

            result[i] += v_p_value + v_d_value;

            _pids[arm][i].vel_error_last = vel_error;
        }
    }
    return result;
}

void KrangControl::initPIDs(lwa_arm_t arm) {
	bool use_pos[] = {true, true, true, true, true, true, true};
	bool use_vel[] = {true, true, true, true, true, true, true};
	double init_K_p_p[] = {5.0,  15.0, 15.0, 12.0, 15.0,  7.0,  7.0};
	double init_K_p_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
	double init_K_v_p[] = {1.0,   1.0,  1.0,  1.0,  1.0,  1.0,  1.0};
	// double init_K_v_p[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
	double init_K_v_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};

	_pids.resize(2);
    for(int i = 0; i < _arm_motors[arm].n; i++) {
        _pids[arm][i].pos_target = 0.0;
        _pids[arm][i].vel_target = 0.0;
        _pids[arm][i].pos_error_last = 0.0;
        _pids[arm][i].vel_error_last = 0.0;
        _pids[arm][i].pos_target = _arm_motors[arm].pos[i];

        _pids[arm][i].K_p_p = init_K_p_p[i];
		_pids[arm][i].K_p_d = init_K_p_d[i];
		_pids[arm][i].K_v_p = init_K_v_p[i];
		_pids[arm][i].K_v_d = init_K_v_d[i];
		_pids[arm][i].use_pos = use_pos[i];
		_pids[arm][i].use_vel = use_vel[i];
    }
}

void KrangControl::setPIDQRef(lwa_arm_t arm, Eigen::VectorXd& q) {
	for (int i=0; i<_arm_motors[arm].n; i++)
		_pids[arm][i].pos_target = q[i];
}

void KrangControl::setPIDQdotRef(lwa_arm_t arm, Eigen::VectorXd& qdot) {
	for (int i=0; i<_arm_motors[arm].n; i++)
		_pids[arm][i].vel_target = qdot[i];
}

Eigen::VectorXd KrangControl::getPIDQref(lwa_arm_t arm) {
	Eigen::VectorXd qref(7);
	for (int i=0; i<7; i++)
		qref[i] = _pids[arm][i].pos_target;
	return qref;
}

#endif // Experimental
