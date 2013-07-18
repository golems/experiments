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

//#include "WorkspaceControl.h" // just for lwa_arm_t

//TODO: should dump this in a namespace
typedef enum arm {
	LEFT_ARM = 0,
	RIGHT_ARM
} lwa_arm_t;

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
	int initialize(simulation::World* world, somatic_d_t *daemon_cx,
			const char *robot_name = "Krang", bool fake_ctl_mode = false);

	// state update methods
	void updateKrangSkeleton();
	Eigen::VectorXd getArmConfig(lwa_arm_t arm);
	void setArmConfig(lwa_arm_t arm, Eigen::VectorXd &config);
	Eigen::Matrix4d getEffectorPose(lwa_arm_t arm);

	// control methods
	void setControlMode(bool mode);
	void setRobotArmVelocities(lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);
	void setRobotiqGripperAction(lwa_arm_t arm, const Eigen::VectorXi &buttons);
	void halt();

protected:
	// control mode
	bool fake_ctl_mode;
	bool initialized;

	// IMU stuff
	ach_channel_t imu_chan;
	filter_kalman_t *kf;					///< the kalman filter to smooth the imu readings
	double imu_angle;
	double imu_speed;

	// initialization helpers
	void setDartIDs(); ///< sets all relevant skeleton Dof IDs for the robot
	void initArm(lwa_arm_t arm, const char* armName);
	void haltArm(lwa_arm_t arm);
	void initWaist();
	void initTorso();
	void initSchunkGripper(lwa_arm_t gripper, const char* name);
	void initRobotiqGripper(lwa_arm_t arm, const char *chan);
	void initIMU(ach_channel_t &imu_chan);

	// STATIC update method helpers (TODO make static)
	void updateRobotSkelFromSomaticMotor(somatic_motor_t &mot, std::vector<int> &IDs);
	void updateRobotSkelFromSomaticWaist();
	void updateRobotSkelFromIMU();
	void getIMU();

	// control helpers
	void sendRobotArmVelocities(lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);
	void fakeArmMovement(lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);

private:
	// All important pointers
	simulation::World* _world;
	dynamics::SkeletonDynamics* _krang;
	std::vector<kinematics::BodyNode*> _gripperNodes;

	// dart IDs
	std::vector< std::vector<int> > _armIDs;
	std::vector<int> _imuIDs;
	std::vector<int> _waistIDs;
	std::vector<int> _torsoIDs;

	// somatic types
	somatic_motor_t _waist;	///< waist motor
	somatic_motor_t _torso;	///< motor motor
	somatic_d_t *_daemon_cx;///< somatic daemon pointer
	std::vector<somatic_motor_t> _arm_motors; ///< arm motors
	std::vector<somatic_motor_t> _schunk_gripper_motors; ///< gripper motors
	std::vector<ach_channel_t> _robotiq_gripper_channels; ///< ach channels for robotiq
};

#endif /* KRANGCONTROL_H_ */
