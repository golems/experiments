/*
 * KrangControl.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#include <unistd.h>
#include <imud.h>
#include "KrangControl.h"

KrangControl::KrangControl() {}

KrangControl::~KrangControl() {
	// TODO Auto-generated destructor stub
}

int KrangControl::initialize(simulation::World* world, somatic_d_t *daemon_cx, bool fake_ctl_mode) {
	this->daemon_cx = daemon_cx;
	this->arm_motors.resize(2);
	this->armIDs.resize(2);
	this->schunk_gripper_motors.resize(2);
	this->robotiq_gripper_channels.resize(2);
	setDartIDs(world);
	setControlMode(fake_ctl_mode);
	return 1;
}

void KrangControl::setControlMode(bool fake_ctl_mode) {
	this->fake_ctl_mode = fake_ctl_mode;

	if (!this->fake_ctl_mode && !initialized) {
		initArm(*daemon_cx, arm_motors[LEFT_ARM], "llwa");
		initArm(*daemon_cx, arm_motors[RIGHT_ARM], "rlwa");
		//initSchunkGripper(*daemon_cx, schunk_gripper_motors[LEFT_ARM], "lGripper");
		//initSchunkGripper(*daemon_cx, schunk_gripper_motors[RIGHT_ARM], "rGripper");
		initRobotiqGripper(LEFT_ARM, "lgripper-cmd");
		initRobotiqGripper(RIGHT_ARM, "rgripper-cmd");
		initWaist(*daemon_cx, waist);
		initIMU(*daemon_cx, imu_chan);

		initialized = true;
	}
}

/*
 * Reads the full Krang configuration from somatinc and updates the krang skeleton in the
 * provided world.
 */
void KrangControl::updateKrangSkeleton(simulation::World* world) {

	updateRobotSkelFromSomaticMotor(world, *daemon_cx, arm_motors[LEFT_ARM], armIDs[LEFT_ARM]);
	updateRobotSkelFromSomaticMotor(world, *daemon_cx, arm_motors[RIGHT_ARM], armIDs[RIGHT_ARM]);
	updateRobotSkelFromSomaticWaist(world, *daemon_cx, waist, waistIDs);
	updateRobotSkelFromIMU(world);
}

Eigen::VectorXd KrangControl::getArmConfig(simulation::World* world, lwa_arm_t arm) {
	return world->getSkeleton("Krang")->getConfig(armIDs[arm]);
}

void KrangControl::setArmConfig(simulation::World* world, lwa_arm_t arm, Eigen::VectorXd &config) {
	world->getSkeleton("Krang")->setConfig(armIDs[arm], config);
}

void KrangControl::sendRobotArmVelocities(lwa_arm_t arm, Eigen::VectorXd& qdot, double dt) {

	//somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, qdot.data()*dt, 7, NULL);

	double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for(size_t i = 0; i < 7; i++)
		dq[i] = qdot(i) * dt;
	somatic_motor_cmd(daemon_cx, &arm_motors[arm], SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
}

void KrangControl::setRobotArmVelocities(simulation::World* world,
		lwa_arm_t arm, Eigen::VectorXd& qdot, double dt) {

	if (fake_ctl_mode)
		fakeArmMovement(world, arm, qdot, dt);
	else
		sendRobotArmVelocities(arm, qdot, dt);
}

void KrangControl::initRobotiqGripper(lwa_arm_t arm, const char *chan) {
    // for robotiq we currently talk over ach directly
    ach_open(&robotiq_gripper_channels[arm], chan, NULL);
}

void KrangControl::setRobotiqGripperAction(lwa_arm_t arm, const Eigen::VectorXi& buttons) {

	// super basic open and close actions, for now...

	robotiqd_achcommand_t rqd_msg;
	rqd_msg.mode = GRASP_BASIC;
	rqd_msg.grasping_speed = 0xff;
	rqd_msg.grasping_force = 0xff;

	if (buttons[0]) rqd_msg.grasping_pos = 0x00;
	if (buttons[1]) rqd_msg.grasping_pos = 0xff;

	if (buttons[0] || buttons[1])
		ach_put(&robotiq_gripper_channels[arm], &rqd_msg, sizeof(rqd_msg));
}


void KrangControl::fakeArmMovement(simulation::World* world, lwa_arm_t arm, Eigen::VectorXd& qdot, double dt) {

	Eigen::VectorXd q = world->getSkeleton("Krang")->getConfig(armIDs[arm]);
	q += qdot * dt;
	world->getSkeleton("Krang")->setConfig(armIDs[arm], q);
}

void KrangControl::halt() {

	haltArm(*daemon_cx, arm_motors[LEFT_ARM]);
	haltArm(*daemon_cx, arm_motors[RIGHT_ARM]);
	initialized = false;
}

//double KrangControl::ssdmu_pitch(double x, double y, double z) {
//
//    double newX;
//    newX = x*cos(csr) - y*sin(csr);
//    return atan2(newX, z);
//}

void KrangControl::setDartIDs(simulation::World* world) {
	// IDs for various krang body parts, hard-coded for now

	int idL[] = {11, 13, 15, 17, 19, 21, 23};
	armIDs[LEFT_ARM] = std::vector<int>(idL, idL + sizeof(idL)/sizeof(idL[0]));

	int idR[] = {12, 14, 16, 18, 20, 22, 24};
	armIDs[RIGHT_ARM] = std::vector<int>(idR, idR + sizeof(idR)/sizeof(idR[0]));

	int imu_ids[] = {5};
	imuIDs = std::vector<int>(imu_ids, imu_ids + sizeof(imu_ids)/sizeof(imu_ids[0]));

	int waist_ids[] = {8};
	waistIDs = std::vector<int>(waist_ids, waist_ids + sizeof(waist_ids)/sizeof(waist_ids[0]));
}

void KrangControl::updateRobotSkelFromSomaticMotor(simulation::World* world,
		somatic_d_t& daemon_cx, somatic_motor_t& mot, std::vector<int>& IDs) {

	assert(mot.n == IDs.size());
	Eigen::VectorXd vals(mot.n);
	somatic_motor_update(&daemon_cx, &mot);
	for(size_t i = 0; i < mot.n; i++)
		vals(i) = mot.pos[i];
	world->getSkeleton("Krang")->setConfig(IDs, vals);
}

void KrangControl::updateRobotSkelFromSomaticWaist(simulation::World* world,
		somatic_d_t& daemon_cx, somatic_motor_t& waist,
		std::vector<int>& waistIDs) {

	// Read the waist's state and update the averaged waist position
    somatic_motor_update(&daemon_cx, &waist);
    double waist_angle = (waist.pos[0] - waist.pos[1]) / 2.0;

    Eigen::VectorXd vals(1);
    vals << waist_angle;
    world->getSkeleton("Krang")->setConfig(waistIDs, vals);
}

void KrangControl::initArm(somatic_d_t& daemon_cx, somatic_motor_t& arm, const char* armName) {

	// Get the channel names
	char cmd_name [16], state_name [16];
	sprintf(cmd_name, "%s-cmd", armName);
	sprintf(state_name, "%s-state", armName);

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(&daemon_cx, &arm, 7, cmd_name, state_name);
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&arm.pos_valid_min, &arm.vel_valid_min,
		&arm.pos_limit_min, &arm.pos_limit_min,
		&arm.pos_valid_max, &arm.vel_valid_max,
		&arm.pos_limit_max, &arm.pos_limit_max};
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);

	// Update and reset them
	somatic_motor_update(&daemon_cx, &arm);
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
	usleep(1e5);
}

void KrangControl::haltArm(somatic_d_t& daemon_cx, somatic_motor_t& arm) {

	double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
}

void KrangControl::initIMU(somatic_d_t& daemon_cx, ach_channel_t& imu_chan) {

	somatic_d_channel_open(&daemon_cx, &imu_chan, "imu-data", NULL);

	// Set the offset values to amc motor group so initial wheel pos readings are zero
	getIMU();

	usleep(1e5);

	// Initialize kalman filter for the imu and set the measurement and meas. noise matrices
	// Also, set the initial reading to the current imu reading to stop moving from 0 to current
	kf = new filter_kalman_t;
	filter_kalman_init(kf, 2, 0, 2);
	kf->C[0] = kf->C[3] = 1.0;
	kf->Q[0] = kf->Q[3] = 1e-3;
	kf->x[0] = imu_angle, kf->x[1] = imu_speed;
}

void KrangControl::initWaist(somatic_d_t& daemon_cx, somatic_motor_t& waist) {

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(&daemon_cx, &waist, 2, "waist-cmd", "waist-state");
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&waist.pos_valid_min, &waist.vel_valid_min,
		&waist.pos_limit_min, &waist.pos_limit_min,
		&waist.pos_valid_max, &waist.vel_valid_max,
		&waist.pos_limit_max, &waist.pos_limit_max};
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 2);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 2);

	// Update and reset them
	somatic_motor_update(&daemon_cx, &waist);
	somatic_motor_cmd(&daemon_cx, &waist, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 2, NULL);
}

void KrangControl::initTorso(somatic_d_t& daemon_cx, somatic_motor_t& torso) {

	// Initialize the arm with the daemon context, channel names and # of motors
	somatic_motor_init(&daemon_cx, &torso, 1, "torso-cmd", "torso-state");
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&torso.pos_valid_min, &torso.vel_valid_min,
		&torso.pos_limit_min, &torso.pos_limit_min,
		&torso.pos_valid_max, &torso.vel_valid_max,
		&torso.pos_limit_max, &torso.pos_limit_max};
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 1);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 1);

	// Update and reset them
	somatic_motor_update(&daemon_cx, &torso);
	somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
}

void KrangControl::initSchunkGripper(somatic_d_t& daemon_cx, somatic_motor_t& gripper, const char* name) {

	// Get the channel names
	char cmd_name [16], state_name [16];
	sprintf(cmd_name, "%s-cmd", name);
	sprintf(state_name, "%s-state", name);

	// Create the motor reference for the left gripper
	somatic_motor_init(&daemon_cx, &gripper, 1, cmd_name, state_name);
	usleep(1e5);

	// Set the min/max values for valid and limits values
	aa_fset(gripper.pos_valid_min, 0.009, 1);
	aa_fset(gripper.pos_limit_min, 0.009, 1);
	aa_fset(gripper.pos_valid_max, 0.068, 1);
	aa_fset(gripper.pos_limit_max, 0.068, 1);
	aa_fset(gripper.vel_valid_min, -0.008, 1);
	aa_fset(gripper.vel_limit_min, -0.008, 1);
	aa_fset(gripper.vel_valid_max, 0.008, 1);
	aa_fset(gripper.vel_limit_max, 0.008, 1);

	// Update and reset them
	somatic_motor_update(&daemon_cx, &gripper);
	somatic_motor_cmd(&daemon_cx, &gripper, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
	usleep(1e5);
}

void KrangControl::updateRobotSkelFromIMU(simulation::World* world) {
	dynamics::SkeletonDynamics* krang = world->getSkeleton("Krang");

	Eigen::VectorXd imu_pos(1);
	getIMU();
	imu_pos << -imu_angle + M_PI/2;
	krang->setConfig(imuIDs, imu_pos);
}


/// Computes the imu value from the imu readings
void KrangControl::getIMU() {

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
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imu_chan, &abstime);
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	imu_angle = atan2(newX, imu_msg->data[2]);
	imu_speed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// ======================================================================
	// Filter the readings

	// Skip if a filter is not provided
	if(kf == NULL) return;

	// Setup the data
	kf->z[0] = imu_angle, kf->z[1] = imu_speed;

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
	imu_angle = kf->x[0], imu_speed = kf->x[1];
}


/*
 * ENDSANDBOX
 */

