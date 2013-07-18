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

	// Robot initialization methods
	int initialize(simulation::World* world, somatic_d_t *daemon_cx,
			const char *robot_name = "Krang", bool fake_ctl_mode = false);
	void setControlMode(bool mode);

	// Robot update methods
	void updateKrangSkeleton();
	void setArmConfig(lwa_arm_t arm, Eigen::VectorXd &config);
	void updateFTOffset(lwa_arm_t arm);

	// Robot query methods
	Eigen::VectorXd getArmConfig(lwa_arm_t arm);
	Eigen::Matrix4d getEffectorPose(lwa_arm_t arm);		///< returns an effector world pose
	Eigen::MatrixXd getEffectorJacobian(lwa_arm_t arm);	///< returns an effector jacobian
	Eigen::VectorXd getFtWorldWrench(lwa_arm_t arm);	///< returns an effector force-sensor wrench in world coords

	// Robot control methods
	void setRobotArmVelocities(lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);
	void setRobotiqGripperAction(lwa_arm_t arm, const Eigen::VectorXi &buttons);
	void halt();

protected:
	// control mode
	bool fake_ctl_mode;
	bool initialized;

	// initialization helpers
	void setDartIDs(); ///< sets all relevant skeleton Dof IDs for the robot
	void initWaist();
	void initTorso();
	void initIMU();
	void initFT();
	void haltArm(lwa_arm_t arm);
	void initArm(lwa_arm_t arm, const char* armName);
	void initSchunkGripper(lwa_arm_t gripper, const char* name);
	void initRobotiqGripper(lwa_arm_t arm, const char *chan);

	// Update method helpers
	void updateRobotSkelFromSomaticMotor(somatic_motor_t &mot, std::vector<int> &IDs);
	void updateRobotSkelFromSomaticWaist();
	void updateRobotSkelFromIMU();
	void getIMU();

	// force-torque helpers
	Eigen::VectorXd getFT(lwa_arm_t arm);
	Eigen::VectorXd getExpectedGripperFTWrench(lwa_arm_t arm); ///< models off the gripper from the FT sensor

	// control helpers
	void sendRobotArmVelocities(lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);
	void fakeArmMovement(lwa_arm_t arm, Eigen::VectorXd &qdot, double dt);

private:
	// Dart related pointers
	simulation::World* _world;
	dynamics::SkeletonDynamics* _krang;
	std::vector<kinematics::BodyNode*> _gripper_nodes;

	// dart IDs
	std::vector< std::vector<int> > _arm_ids;
	std::vector<int> _imu_ids;
	std::vector<int> _waist_ids;
	std::vector<int> _torso_ids;

	// somatic types
	somatic_motor_t _waist;	///< waist motor
	somatic_motor_t _torso;	///< motor motor
	somatic_d_t *_daemon_cx;///< somatic daemon pointer
	std::vector<somatic_motor_t> _arm_motors; ///< arm motors
	std::vector<somatic_motor_t> _schunk_gripper_motors; ///< gripper motors
	std::vector<ach_channel_t> _robotiq_gripper_channels; ///< ach channels for robotiq

	// IMU stuff
	ach_channel_t _imu_chan;
	filter_kalman_t *_imu_kf; 		///< the kalman filter to smooth the imu readings
	double _imu_angle;
	double _imu_speed;

	// FT stuff
	std::vector<ach_channel_t> _ft_channels;
	std::vector<Eigen::VectorXd> _ft_offsets;
	static const int _ft_init_iters = 100;
	filter_kalman_t *ft_kf;			///< the kalman filter to smooth the force sensor readings
	Eigen::Vector3d robotiq_com;	///<
	static const double end_effector_mass = 2.3 + 0.169 + 0.000; ///< mass of the end effector

};

#endif /* KRANGCONTROL_H_ */
