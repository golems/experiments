/*
 * channel.h
 *
 *  Created on: Jul 14, 2013
 *      Author: jscholz
 */

#ifndef CHANNEL_H_
#define CHANNEL_H_


#include <amino.h>
#include "initModules.h"
#include <robotiqd.h>

// somatic globals
somatic_d_t daemon_cx;
somatic_motor_t llwa;
somatic_motor_t rlwa;
somatic_motor_t waist;

// gripper stuff
ach_channel_t lgripper_chan;
ach_channel_t rgripper_chan;

// dart IDs
vector<int> armIDsL;
vector<int> armIDsR;
std::vector<int> imuIDs;
vector<int> waistIDs;

// IMU stuff
double csr = -.7853981634; // Angle that the DMU is mounted at: 45 degrees.
ach_channel_t imu_chan;

void ach_init(ach_channel_t* chan, char* chan_name, uint8_t *achbuf, size_t n_achbuf) {
	// Set up the buffer
	achbuf = AA_NEW_AR(uint8_t,  n_achbuf );

	// Open the given channel
	int r = ach_open( chan, chan_name, NULL );
	aa_hard_assert( ACH_OK == r, "Couldn't open channel %s\n", chan_name );
	r = ach_flush( chan );
	aa_hard_assert( ACH_OK == r, "Couldn't flush channel\n");
}

// helper function for getIMUPitch
double ssdmu_pitch(double x, double y, double z) {
    double newX;
    newX = x*cos(csr) - y*sin(csr);
    return atan2(newX, z);
}

/*
 * Returns the angle reported by the IMU in frame that DART can understand. Does not do any filtering.
 */
double getIMUPitch() {
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

/*
 * Send joint velocities to robot over somatic
 */
void sendRobotArmVelocities(somatic_d_t &daemon_cx, somatic_motor_t &arm, VectorXd &qdot,
		double dt) {

	//somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, qdot.data()*dt, 7, NULL);

	double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for(size_t i = 0; i < 7; i++)
		dq[i] = qdot(i) * dt;
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
	//somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, dq, 7, NULL);
}

/*
 * Opens or closes the target gripper according to the spacenav button state
 */
void handleSpacenavButtons(const VectorXi &buttons, ach_channel_t &grip_chan) {
	robotiqd_achcommand_t rqd_msg;
	rqd_msg.mode = GRASP_BASIC;
	rqd_msg.grasping_speed = 0xff;
	rqd_msg.grasping_force = 0xff;

	if (buttons[0]) rqd_msg.grasping_pos = 0x00;
	if (buttons[1]) rqd_msg.grasping_pos = 0xff;

	if (buttons[0] || buttons[1])
		ach_put(&grip_chan, &rqd_msg, sizeof(rqd_msg));
}

/*
 * Hard-coded IDs for various krang body parts
 */
void setDartIDs(simulation::World* world) {
	int idL[] = {11, 13, 15, 17, 19, 21, 23};
	armIDsL = vector<int>(idL, idL + sizeof(idL)/sizeof(idL[0]));

	int idR[] = {12, 14, 16, 18, 20, 22, 24};
	armIDsR = vector<int>(idR, idR + sizeof(idR)/sizeof(idR[0]));

	int imu_ids[] = {5};
	imuIDs = vector<int>(imu_ids, imu_ids + sizeof(imu_ids)/sizeof(imu_ids[0]));

	int waist_ids[] = {8};
	waistIDs = vector<int>(waist_ids, waist_ids + sizeof(waist_ids)/sizeof(waist_ids[0]));
}


/*
 * Updates motor body configurations (e.g. arms, waist, torso, etc) in krang
 * skeleton from current somatic motor states
 */
void updateRobotSkelFromSomaticMotor(simulation::World* world, somatic_d_t &daemon_cx,
		somatic_motor_t &mot, vector<int> &IDs) {

	assert(mot.n == IDs.size());
	VectorXd vals(mot.n);
	somatic_motor_update(&daemon_cx, &mot);
	for(size_t i = 0; i < mot.n; i++)
		vals(i) = mot.pos[i];
	world->getSkeleton("Krang")->setConfig(IDs, vals);
}

void updateRobotSkelFromSomaticWaist(simulation::World* world, somatic_d_t &daemon_cx,
		somatic_motor_t &waist, vector<int> &waistIDs) {

	// Read the waist's state and update the averaged waist position
    somatic_motor_update(&daemon_cx, &waist);
    double waist_angle = (waist.pos[0] - waist.pos[1]) / 2.0;

    VectorXd vals(1);
    vals << waist_angle;
    world->getSkeleton("Krang")->setConfig(waistIDs, vals);
}

void updateRobotSkelFromIMU(simulation::World* world) {
	dynamics::SkeletonDynamics* krang = world->getSkeleton("Krang");

	Eigen::VectorXd imu_pos(1);
	imu_pos << getIMUPitch();
	krang->setConfig(imuIDs, imu_pos);
}

#endif /* CHANNEL_H_ */
