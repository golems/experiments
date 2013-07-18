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
#include <filter.h>

#include <Eigen/Dense>
#include <simulation/World.h>

#include "WorkspaceControl.h" // just for lwa_arm_t

///< An enum for controlling whether we're dispatching actual motor commands or faking it
//typedef enum CTL_MODES {
//	FAKE= 0,
//	REAL
//} ctl_mode_t;

/*
 * This class bundles all the variables and methods necessary to control Krang
 * over somatic into a single class.
 */
class KrangControl {
public:
	KrangControl();
	virtual ~KrangControl();

	// initialization methods
	int initialize(simulation::World* world, somatic_d_t *daemon_cx, bool fake_ctl_mode = false);

	// state update methods
	void updateKrangSkeleton(simulation::World* world);
	Eigen::VectorXd getArmConfig(simulation::World* world, lwa_arm_t arm);
	void setArmConfig(simulation::World* world, lwa_arm_t arm, Eigen::VectorXd &config);

	// control methods
	void setControlMode(bool mode);
	void setRobotArmVelocities(simulation::World* world, lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);
	void setRobotiqGripperAction(lwa_arm_t arm, const Eigen::VectorXi &buttons);
	void halt();

protected:
	// control mode
	bool fake_ctl_mode;
	bool initialized;

	// somatic globals
	somatic_d_t *daemon_cx;  ///< somatic daemon pointer
	somatic_motor_t waist;	///< waist motor
	somatic_motor_t torso;	///< motor motor
	std::vector<somatic_motor_t> arm_motors; ///< arm motors
	std::vector<somatic_motor_t> schunk_gripper_motors; ///< gripper motors
	std::vector<ach_channel_t> robotiq_gripper_channels; ///< ach channels for robotiq

	// dart IDs
	std::vector< std::vector<int> > armIDs;
	std::vector<int> imuIDs;
	std::vector<int> waistIDs;

	// IMU stuff
	ach_channel_t imu_chan;
	filter_kalman_t *kf;					///< the kalman filter to smooth the imu readings
	double imu_angle;
	double imu_speed;
	//double ssdmu_pitch(double x, double y, double z); ///< helper for converting IMU data to a scalar pitch


	// initialization helpers
	void setDartIDs(simulation::World* world); ///< sets all relevant skeleton Dof IDs for the robot
	void initArm (somatic_d_t& daemon_cx, somatic_motor_t& arm, const char* armName);
	void haltArm(somatic_d_t &daemon_cx, somatic_motor_t &arm);
	void initWaist(somatic_d_t& daemon_cx, somatic_motor_t& waist);
	void initTorso (somatic_d_t& daemon_cx, somatic_motor_t& torso);
	void initSchunkGripper(somatic_d_t& daemon_cx, somatic_motor_t& gripper, const char* name);
	void initRobotiqGripper(lwa_arm_t arm, const char *chan);
	void initIMU(somatic_d_t& daemon_cx, ach_channel_t &imu_chan);

	// update method helpers
	//double getIMUPitch();
	void getIMU();
	void updateRobotSkelFromSomaticMotor(simulation::World* world, somatic_d_t &daemon_cx, somatic_motor_t &mot, std::vector<int> &IDs);
	void updateRobotSkelFromSomaticWaist(simulation::World* world, somatic_d_t &daemon_cx, somatic_motor_t &waist, std::vector<int> &waistIDs);
	void updateRobotSkelFromIMU(simulation::World* world);

	// control helpers
	void sendRobotArmVelocities(lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);
	void fakeArmMovement(simulation::World* world, lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);
};

#endif /* KRANGCONTROL_H_ */
