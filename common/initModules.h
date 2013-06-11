/**
 * @file initModules.h
 * @author Can Erdogan
 * @date May 24, 2013
 * @brief This file contains the small code snippets that maybe shared between the experiments.
 * For instance, the code to initialize the arms or the grippers would be used across kinematics
 * and manipulation.
 */

#pragma once

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

/* ********************************************************************************************** */
/// Initializes a daemon with the given name
void initDaemon (somatic_d_t& daemon_cx, char* name) {
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "bal-hack";
	somatic_d_init( &daemon_cx, &dopt );
}

/* ********************************************************************************************** */
/// Initializes the gripper with the given name: either "lgripper" or "rgripper"
void initGripper (somatic_d_t& daemon_cx, somatic_motor_t& gripper, const char* name) {	

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

/* ********************************************************************************************** */
/// Initializes the arm with the given name: either "llwa" or "rlwa".
void initArm (somatic_d_t& daemon_cx, somatic_motor_t& arm, const char* armName) {	

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
}
/* ********************************************************************************************** */
