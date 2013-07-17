/*
 * KrangControl.h
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#ifndef KRANGCONTROL_H_
#define KRANGCONTROL_H_

#include <amino.h>
#include <ach.h>
#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic/motor.h>
#include <somatic.pb-c.h>
#include <robotiqd.h>

#include <Eigen/Dense>
#include <simulation/World.h>

/*
 * This class bundles all the variables and methods necessary to control Krang
 * over somatic into a single class.
 */
class KrangControl {
public:
	KrangControl();
	virtual ~KrangControl();

	// initialization methods
	int initialize(somatic_d_t *daemon_cx); ///< initializes full robot

	// state update methods
	void updateKrangSkeleton(simulation::World* world);

	// control methods
	void sendRobotArmVelocities(somatic_motor_t &arm, Eigen::VectorXd &qdot, double dt);
	void halt();

protected:
	// somatic globals
	somatic_d_t *daemon_cx;  ///< somatic daemon pointer
	somatic_motor_t llwa;	///< left arm motor
	somatic_motor_t rlwa;	///< right arm motor
	somatic_motor_t waist;	///< waist motor
	somatic_motor_t torso;	///< motor motor

	// gripper stuff
	ach_channel_t lgripper_chan;
	ach_channel_t rgripper_chan;

	// dart IDs
	std::vector<int> armIDsL;
	std::vector<int> armIDsR;
	std::vector<int> imuIDs;
	std::vector<int> waistIDs;

	// IMU stuff
	static const double csr = -.7853981634; // Angle that the DMU is mounted at: 45 degrees.
	ach_channel_t imu_chan;
	double ssdmu_pitch(double x, double y, double z); ///< helper for converting IMU data to a scalar pitch

	// initialization helpers
	void setDartIDs(simulation::World* world); ///< sets all relevant skeleton Dof IDs for the robot
	void initArm (somatic_d_t& daemon_cx, somatic_motor_t& arm, const char* armName);
	void haltArm(somatic_d_t &daemon_cx, somatic_motor_t &arm);
	void initIMU(somatic_d_t& daemon_cx, ach_channel_t &imu_chan);
	void initWaist(somatic_d_t& daemon_cx, somatic_motor_t& waist);
	void initTorso (somatic_d_t& daemon_cx, somatic_motor_t& torso);
	void initGripper (somatic_d_t& daemon_cx, somatic_motor_t& gripper, const char* name);

	// update method helpers
	double getIMUPitch();
	void updateRobotSkelFromSomaticMotor(simulation::World* world, somatic_d_t &daemon_cx, somatic_motor_t &mot, std::vector<int> &IDs);
	void updateRobotSkelFromSomaticWaist(simulation::World* world, somatic_d_t &daemon_cx, somatic_motor_t &waist, std::vector<int> &waistIDs);
	void updateRobotSkelFromIMU(simulation::World* world);

};

#endif /* KRANGCONTROL_H_ */
