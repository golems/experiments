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
/** \file KrangControl.h
 *
 *  \author Jonathan Scholz
 *  \date 7/17/2013
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

//#define EXPERIMENTAL

//TODO: should dump this in a namespace
typedef enum arm {
	LEFT_ARM = 0,
	RIGHT_ARM
} lwa_arm_t;


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
			const char *robot_name = "Krang");
	void initSomatic(); ///< initializes all of krangs body parts over somatic

	// Robot update methods
	bool updateKrangSkeleton();
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
	void resetMotors();
	void setMotorOutputMode(bool mode);

	dynamics::SkeletonDynamics* getKrang() const { return _krang; }

protected:
	// control mode
	bool send_motor_cmds;
	bool initialized;

	// initialization helpers
	void setDartIDs(); ///< sets all relevant skeleton Dof IDs for the robot
	void initWaist();
	void initTorso();
	void initIMU();
	void initFT();
	void initArm(lwa_arm_t arm, const char* armName);
	void initSchunkGripper(lwa_arm_t gripper, const char* name);
	void initRobotiqGripper(lwa_arm_t arm, const char *chan);

	// Update method helpers
	bool updateRobotSkelFromSomaticMotor(somatic_motor_t &mot, std::vector<int> &IDs);
	void updateRobotSkelFromSomaticWaist();
	void updateRobotSkelFromIMU();
	void getIMU();

	// force-torque helpers
	Eigen::VectorXd getFT(lwa_arm_t arm, bool wait = false);

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
	//filter_kalman_t *ft_kf;		///< the kalman filter to smooth the force sensor readings
	Eigen::Vector3d _robotiq_com;	///<
	static constexpr double _robotiq_mass = 2.3 + 0.169 + 0.000; ///< mass of the end effector (gripper + collar + sensor)


	/*
	 * STILL EXPERIMENTAL!
	 */
	bool _current_mode;
#ifdef EXPERIMENTAL
public:
	// current control stuff
	Eigen::VectorXd updatePIDs(lwa_arm_t arm);
	void initPIDs(lwa_arm_t arm);
	void setPIDQRef(lwa_arm_t arm, Eigen::VectorXd &q);
	void setPIDQdotRef(lwa_arm_t arm, Eigen::VectorXd &qdot);
	Eigen::VectorXd getPIDQref(lwa_arm_t arm);

	/* a data type for holding on to the state of a single motor's PID controller */
	typedef struct {
	    bool use_pos;
	    bool use_vel;

	    double pos_target;
	    double vel_target;

	    double output;

	    double K_p_p;
	    double K_p_d;
	    double K_v_p;
	    double K_v_d;
	    double pos_error_last;
	    double vel_error_last;
	} pid_state_t;
	// PID controller values
	std::vector<pid_state_t[7]> _pids;
#endif

};
#endif /* KRANGCONTROL_H_ */
