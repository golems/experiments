/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file kore.cpp
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The main source file for the Krang support file which has the definition for the 
 * KHardware struct constructor.
 */

#include "kore.h"
#include <unistd.h>
#include "display.hpp"

using namespace Eigen;

namespace Krang {
	
/* ******************************************************************************************** */
KHardware::KHardware (Mode mode_, somatic_d_t* daemon_cx_, dynamics::SkeletonDynamics* robot_) {

	// Set the local variables
	daemon_cx = daemon_cx_;
	robot = robot_;
	mode = mode_;

	// Initialize all the 'optional' pointers to nulls and sanity check the inputs
	lft = rft = NULL;
	amc = larm = rarm = torso = lgripper = rgripper = NULL;
	waistCmdChan = NULL;
	assert((daemon_cx != NULL) && "Can not give null daemon context to hardware constructor");
	assert((robot != NULL) && "Can not give null dart kinematics to hardware constructor");
	
	// Initialize the imu sensor and average the first 500 values
	initImu();

	// Define the pos/vel limits for the motor groups
	VectorXd lim7 = VectorXd::Ones(7) * 1024.1;
	VectorXd lim4 = VectorXd::Ones(4) * 1024.1; 
	VectorXd lim2 = VectorXd::Ones(2) * 1024.1; 
	VectorXd lim1 = VectorXd::Ones(1) * 1024.1;

	// Initialize the Schunk (+ Robotiq) motor groups
	if(mode & MODE_LARM) 
		initMotorGroup(larm, "llwa-cmd", "llwa-state", -lim7, lim7, -lim7, lim7);	
	if(mode & MODE_RARM) 
		initMotorGroup(rarm, "rlwa-cmd", "rlwa-state", -lim7, lim7, -lim7, lim7);	
	if(mode & MODE_TORSO) 
		initMotorGroup(torso, "torso-cmd", "torso-state", -lim1, lim1, -lim1, lim1);	
	if((mode & MODE_LARM) && (mode & MODE_GRIPPERS_SCH))
		initMotorGroup(lgripper, "lgripper-cmd", "lgripper-state", -lim1, lim1, -lim1, lim1);
	if((mode & MODE_LARM) && (mode & MODE_GRIPPERS))
		initMotorGroup(lgripper, "lgripper-cmd", "lgripper-state", -lim4, lim4, -lim4, lim4);
	if((mode & MODE_RARM) && (mode & MODE_GRIPPERS_SCH))
		initMotorGroup(rgripper, "rgripper-cmd", "rgripper-state", -lim1, lim1, -lim1, lim1);
	if((mode & MODE_RARM) && (mode & MODE_GRIPPERS))
		initMotorGroup(rgripper, "rgripper-cmd", "rgripper-state", -lim4, lim4, -lim4, lim4);

	// Initialize the wheel motor groups which depend on imu readings to get absolute wheel positions
	if(mode & MODE_AMC) initWheels();

	// Initialize the ach channels to the waist daemon and the waist state channel. The daemon moves
	// the motors in unison with current control and pciod updates their state on the state channel.
	if(mode & MODE_WAIST) initWaist();

	// Updates the robot kinematics
	updateKinematics();
	
	// After initializing the rest of the robot (need kinematics), we can initialize f/t sensors. 
	// TODO: Determine the type of the gripper from the mode
	FT::GripperType ft_grippers;
	if (mode & MODE_GRIPPERS) ft_grippers = FT::GRIPPER_TYPE_ROBOTIQ;
	if (mode & MODE_GRIPPERS_SCH) ft_grippers = FT::GRIPPER_TYPE_SCHUNK;
	if(mode & MODE_LARM) lft = new FT(ft_grippers, daemon_cx, robot, LEFT);
	if(mode & MODE_RARM) rft = new FT(ft_grippers, daemon_cx, robot, RIGHT);
}

/* ******************************************************************************************** */
KHardware::~KHardware () {

	// Close imu channel and the filter
	somatic_d_channel_close(daemon_cx, imu_chan);
	delete imu_chan;
	filter_kalman_destroy(kfImu);	

	// Destroy the ft sensors
	if(lft != NULL) delete lft;
	if(rft != NULL) delete rft;
	
	// Send zero velocity to amc and clean it up
	double zeros2[2] = {0.0, 0.0}, zeros7[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
	if(amc != NULL) {
		somatic_motor_cmd(daemon_cx, amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, zeros2, 2, NULL);
		somatic_motor_destroy(daemon_cx, amc);
		delete amc;
	}

	// Send halt messages directly to the left and right arms
	if(rarm != NULL) {
		somatic_motor_halt(daemon_cx, rarm);
		somatic_motor_destroy(daemon_cx, rarm);
		delete rarm;
	}
	if(larm != NULL) {
		somatic_motor_halt(daemon_cx, larm);
		somatic_motor_destroy(daemon_cx, larm);
		delete larm;
	}
	
	// Destroy the gripper motors
	if(lgripper != NULL) { 
		somatic_motor_halt(daemon_cx, lgripper);
		somatic_motor_destroy(daemon_cx, lgripper);
		delete lgripper;
	}
	if(rgripper != NULL) { 
		somatic_motor_halt(daemon_cx, rgripper);
		somatic_motor_destroy(daemon_cx, rgripper);
		delete rgripper;
	}

	// Stop and destroy	waist motors
	if(waist != NULL) { 

		// Create a waist daemon message with the stop mode
		Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
		somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);

		// Send the message
		int r = SOMATIC_PACK_SEND(waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
		if(ACH_OK != r) fprintf(stderr, "Couldn't send stop message to waist: %s\n", 
			ach_result_to_string(static_cast<ach_status_t>(r)));

		// Clean up the memory
		somatic_motor_destroy(daemon_cx, waist);
		delete waist;
	}
}

/* ******************************************************************************************** */
void KHardware::initWaist () {

	// Initialize the channel to the waist daemon
	waistCmdChan = new ach_channel_t();
	somatic_d_channel_open(daemon_cx, waistCmdChan, "waistd-cmd", NULL);

	// Initialize the somatic motor interface to listen to the state
	waist = new somatic_motor_t();
	somatic_motor_init(daemon_cx, waist, 2, NULL, "waist-state");
	usleep(1e5);

	// Set the min/max values for the pos/vel fields' valid and limit values
	VectorXd lim2 = VectorXd::Ones(2) * 1024.1; 
	for(int i = 0; i < 2; i++) {
		waist->pos_valid_min[i] = -lim2[i];
		waist->pos_limit_min[i] = -lim2[i];
		waist->pos_valid_max[i] = lim2[i];
		waist->pos_limit_max[i] = lim2[i];

		waist->vel_valid_min[i] = -lim2[i];
		waist->vel_limit_min[i] = -lim2[i];
		waist->vel_valid_max[i] = lim2[i];
		waist->vel_limit_max[i] = lim2[i];
	}

	// Get an update
	somatic_motor_update(daemon_cx, waist);
	usleep(1e5);
}

/* ******************************************************************************************** */
void KHardware::updateSensors (double dt) {

	// Update the lower body motors to get the current values
	if(mode & MODE_AMC) somatic_motor_update(daemon_cx, amc);
	if(mode & MODE_TORSO) somatic_motor_update(daemon_cx, torso);
	if(mode & MODE_WAIST) somatic_motor_update(daemon_cx, waist);

	// Update the imu
	getImu(imu_chan, imu, imuSpeed, dt, kfImu); 

	// Update the arms
	if(mode & MODE_LARM) somatic_motor_update(daemon_cx, larm);
	if(mode & MODE_RARM) somatic_motor_update(daemon_cx, rarm);

	// Update the kinematics
	updateKinematics();

	// Update the f/t readings
	if(mode & MODE_LARM) lft->updateExternal();
	if(mode & MODE_RARM) rft->updateExternal();
}

/* ******************************************************************************************** */
void KHardware::updateKinematics () {

	// unify the id vectors so we only have to make a single call
	// TODO: move this to an outside variable and initialize in the
	// the init function
	std::vector<int> all_ids;
	all_ids.insert(all_ids.end(), imuWaist_ids.begin(), imuWaist_ids.end());
	all_ids.insert(all_ids.end(), left_arm_ids.begin(), left_arm_ids.end());
	all_ids.insert(all_ids.end(), right_arm_ids.begin(), right_arm_ids.end());

	// a spot to unify the config vectors for same
	Eigen::VectorXd all_vals(all_ids.size());

	// Update imu and waist values
	assert((mode & MODE_WAIST) && "This code assumes that the robot at least has the waist modules");
	double waist_val = (waist->pos[0] - waist->pos[1]) / 2.0;
	Vector2d imuWaist_vals (-imu + M_PI_2, waist_val);
	for(int i = 0; i < imuWaist_vals.size(); i++) all_vals[i] = imuWaist_vals[i];

	// Update the arms state
	if(mode & MODE_LARM) {
		Vector7d larm_vals = eig7(larm->pos);
		for(int i = 0; i < larm_vals.size(); i++) all_vals[imuWaist_ids.size() + i] = larm_vals[i];
	}
	if(mode & MODE_RARM) {
		Vector7d rarm_vals = eig7(rarm->pos);
		for(int i = 0; i < rarm_vals.size(); i++) all_vals[imuWaist_ids.size() + left_arm_ids.size() + i] = rarm_vals[i];
	}
	robot->setConfig(all_ids, all_vals);
}

/* ******************************************************************************************** */
void KHardware::initImu () {

	// Initialize the ach channel
	imu_chan = new ach_channel_t();
	somatic_d_channel_open(daemon_cx, imu_chan, "imu-data", NULL);

	// Average the first 500 readings
	imu = imuSpeed = 0.0;
	for(int i = 0; i < 500; i++) {
		double tempImu, tempImuSpeed;
		getImu(imu_chan, tempImu, tempImuSpeed, 0.0, NULL); 
		imu += tempImu, imuSpeed += tempImuSpeed;
	}
	imu /= 500.0, imuSpeed /= 500.0;

	// Create the kalman filter
	kfImu = new filter_kalman_t;
	filter_kalman_init(kfImu, 2, 0, 2);
	kfImu->C[0] = kfImu->C[3] = 1.0;
	kfImu->Q[0] = kfImu->Q[3] = 1e-3;
	kfImu->x[0] = imu, kfImu->x[1] = imuSpeed;
}

/* ******************************************************************************************** */
void KHardware::initWheels () {

	// Initialize the motor group (do we need to set any limits?)
	amc = new somatic_motor_t();
	somatic_motor_init(daemon_cx, amc, 2, "amc-cmd", "amc-state");

	// Set the min/max values for the pos/vel fields' valid and limit values
	for(int i = 0; i < 2; i++) {
		amc->pos_valid_min[i] = -1024.1;
		amc->pos_valid_max[i] = 1024.1;
		amc->pos_limit_min[i] = -1024.1;
		amc->pos_limit_max[i] = 1024.1; 

		amc->vel_valid_min[i] = -1024.1;
		amc->vel_valid_max[i] = 1024.1;
		amc->vel_limit_min[i] = -1024.1;
		amc->vel_limit_max[i] = 1024.1;
	}

	// Update and reset them
	somatic_motor_update(daemon_cx, amc);
	usleep(1e5);

	// Set the offset values to amc motor group so initial wheel pos readings are zero
	double pos_offset[2] = {-amc->pos[0] - imu, -amc->pos[1] - imu};
	aa_fcpy(amc->pos_offset, pos_offset, 2);
	usleep(1e5);
}

/* ******************************************************************************************** */
void KHardware::initMotorGroup (somatic_motor_t*& motors, const char* cmd_name, const char* 
		state_name, VectorXd minPos, VectorXd maxPos, VectorXd minVel, VectorXd maxVel) {

	// Initialize the somatic motor struct with the channel names and the number of modules
	motors = new somatic_motor_t();
	size_t numModules = minPos.size();
	assert(minPos.size() == maxPos.size() && minPos.size() == minVel.size() && minPos.size() == maxVel.size());
	somatic_motor_init(daemon_cx, motors, numModules, cmd_name, state_name);

	// Set the min/max values for the pos/vel fields' valid and limit values
	for(int i = 0; i < numModules; i++) {
		motors->pos_valid_min[i] = minPos[i];
		motors->pos_valid_max[i] = maxPos[i];
		motors->pos_limit_min[i] = minPos[i];
		motors->pos_limit_max[i] = maxPos[i];

		motors->vel_valid_min[i] = minVel[i];
		motors->vel_valid_max[i] = maxVel[i];
		motors->vel_limit_min[i] = minVel[i];
		motors->vel_limit_max[i] = maxVel[i];
	}

	// Update and reset them
	somatic_motor_reset(daemon_cx, motors);
	usleep(1e5);
	somatic_motor_update(daemon_cx, motors);
	usleep(1e5);
}

/* ******************************************************************************************** */
void KHardware::printState () {
	VectorXd s = robot->getPose();
	printf("imu: %.3lf, waist: %.3lf, torso: %.3lf\n", s(imuWaist_ids[0]), s(imuWaist_ids[1]), s(9));
	printf("left arm: (");
	for(size_t i = 0; i < 7; i++) printf(" %.3lf,", s(left_arm_ids[i]));
	printf("\b)\nright arm: (");
	for(size_t i = 0; i < 7; i++) printf(" %.3lf,", s(right_arm_ids[i]));
	printf("\b)\n");
}

    /* ******************************************************************************************** */
	void KHardware::printStateCurses(int row, int col) {
		VectorXd s = robot->getPose();
		mvprintw(row, col, "Robot hardware state:");
		mvprintw(row+1, col+1, "imu: %.3lf", s(imuWaist_ids[0]));
		mvprintw(row+1, col+16, "waist: %.3lf", s(imuWaist_ids[1]));
		mvprintw(row+1, col+31, "torso: %.3lf", s(9));
		
		// display amc currents
		char bar_buffer[1024];
		int bar_len = 80;
		double amc_max = 40;
		for(int mot = 0; mot < amc->n; mot++) {
			int i = 0;
			for(; i < bar_len * amc->cur[mot] / amc_max; i++) bar_buffer[bar_len + i] = '|';
			for(; i < bar_len; i++) bar_buffer[bar_len + i] = ' ';
			i = 0;
			for(; i < bar_len * -amc->cur[mot] / amc_max; i++) bar_buffer[bar_len - i] = '|';
			for(; i < bar_len; i++) bar_buffer[bar_len - i] = ' ';
			bar_buffer[2*bar_len-1] = '\0';
			bar_buffer[0] = '[';
			bar_buffer[2*bar_len-2] = ']';
			mvprintw(row+3+mot, col, bar_buffer);
		}

		// print out the left arm, with pretty colors for joint limits
		mvprintw(row+6, col+2, "left arm:");
		kinematics::Dof* dof;
		double dist;
		for(int i = 0; i < 7; i++) {
			dof = robot->getDof(left_arm_ids[i]);
			dist = std::min(std::abs(s[left_arm_ids[i]] - dof->getMin()),
			                std::abs(s[left_arm_ids[i]] - dof->getMax()));
			if (dist < .25) attron(COLOR_PAIR(COLOR_RED_BACKGROUND));
			mvprintw(row+6, col+14+(i*12), "%.8lf", s(left_arm_ids[i]));
			if (dist < .25) attroff(COLOR_PAIR(COLOR_RED_BACKGROUND));
		}

		// the same for the right arm. TODO: unify these
		mvprintw(row+7, col+2, "right arm:");
		for(int i = 0; i < 7; i++) {
			dof = robot->getDof(right_arm_ids[i]);
			dist = std::min(std::abs(s[right_arm_ids[i]] - dof->getMin()),
			                std::abs(s[right_arm_ids[i]] - dof->getMax()));
			if (dist < .25) attron(COLOR_PAIR(COLOR_RED_BACKGROUND));
			mvprintw(row+7, col+14+(i*12), "%.8lf", s(right_arm_ids[i]));
			if (dist < .25) attroff(COLOR_PAIR(COLOR_RED_BACKGROUND));
		}
	}

};	// end of namespace
