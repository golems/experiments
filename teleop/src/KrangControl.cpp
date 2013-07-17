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

int KrangControl::initialize(somatic_d_t *daemon_cx) {
	this->daemon_cx = daemon_cx;
	initArm(*daemon_cx, llwa, "llwa");
	initArm(*daemon_cx, rlwa, "rlwa");
	initWaist(*daemon_cx, waist);
	initIMU(*daemon_cx, imu_chan);
	ach_open(&lgripper_chan, "lgripper-cmd", NULL);
	ach_open(&rgripper_chan, "rgripper-cmd", NULL);
	return 1;
}

/*
 * Reads the full Krang configuration from somatinc and updates the krang skeleton in the
 * provided world.
 */
void KrangControl::updateKrangSkeleton(simulation::World* world) {

	updateRobotSkelFromSomaticMotor(world, *daemon_cx, llwa, armIDsL);
	updateRobotSkelFromSomaticMotor(world, *daemon_cx, rlwa, armIDsR);
	updateRobotSkelFromSomaticWaist(world, *daemon_cx, waist, waistIDs);
	updateRobotSkelFromIMU(world);
}

void KrangControl::sendRobotArmVelocities(somatic_motor_t& arm, Eigen::VectorXd& qdot, double dt) {

	//somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, qdot.data()*dt, 7, NULL);

	double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for(size_t i = 0; i < 7; i++)
		dq[i] = qdot(i) * dt;
	somatic_motor_cmd(daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
	//somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, dq, 7, NULL);
}

void KrangControl::halt() {

	haltArm(*daemon_cx, llwa);
	haltArm(*daemon_cx, rlwa);
}

double KrangControl::ssdmu_pitch(double x, double y, double z) {

    double newX;
    newX = x*cos(csr) - y*sin(csr);
    return atan2(newX, z);
}

void KrangControl::setDartIDs(simulation::World* world) {
	// IDs for various krang body parts, hard-coded for now

	int idL[] = {11, 13, 15, 17, 19, 21, 23};
	armIDsL = std::vector<int>(idL, idL + sizeof(idL)/sizeof(idL[0]));

	int idR[] = {12, 14, 16, 18, 20, 22, 24};
	armIDsR = std::vector<int>(idR, idR + sizeof(idR)/sizeof(idR[0]));

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

void KrangControl::initGripper(somatic_d_t& daemon_cx, somatic_motor_t& gripper, const char* name) {

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

double KrangControl::getIMUPitch() {
    // Get a message
    int r;
    struct timespec timeout = aa_tm_future(aa_tm_sec2timespec(1.0/30.0));
    Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector,
                                                        &protobuf_c_system_allocator,
                                                        IMU_CHANNEL_SIZE,
                                                        &imu_chan,
                                                        &timeout);
    assert((imu_msg != NULL) && "Didn't get IMU message!");

    // extract the data into something we can use
    double imu_sample_x  = imu_msg->data[0];
    double imu_sample_y  = imu_msg->data[1];
    double imu_sample_z  = imu_msg->data[2];

    // Free the unpacked message
    somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

    // compute our result and return it
    //return -ssdmu_pitch(imu_sample_x, imu_sample_y, imu_sample_z) + M_PI/2;
    return 3.4418;
}

void KrangControl::updateRobotSkelFromIMU(simulation::World* world) {
	dynamics::SkeletonDynamics* krang = world->getSkeleton("Krang");
	Eigen::VectorXd imu_pos(1);
	imu_pos << getIMUPitch();
	krang->setConfig(imuIDs, imu_pos);
}
