/**
 * @file initModules.h
 * @author Can Erdogan
 * @date May 24, 2013
 * @brief This file contains the small code snippets that maybe shared between the experiments.
 * For instance, the code to initialize the arms or the grippers would be used across kinematics
 * and manipulation.
 */

#pragma once

#include <unistd.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>
#include <imud.h>

/* ********************************************************************************************* */
/// Initializes a daemon with the given name
static void initDaemon (somatic_d_t& daemon_cx, char* name) {
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "bal-hack";
	somatic_d_init( &daemon_cx, &dopt );
}

/* ********************************************************************************************* */
/// Initializes the gripper with the given name: either "lgripper" or "rgripper"
static void initGripper (somatic_d_t& daemon_cx, somatic_motor_t& gripper, const char* name) {	

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

/* ********************************************************************************************* */
/// Initializes the torso motor
static void initTorso (somatic_d_t& daemon_cx, somatic_motor_t& torso) {

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

/// Initializes the waist motor
static void initWaist(somatic_d_t& daemon_cx, somatic_motor_t& waist) {

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

/* ********************************************************************************************* */
/// Initializes the arm with the given name: either "llwa" or "rlwa".
static void initArm (somatic_d_t& daemon_cx, somatic_motor_t& arm, const char* armName) {	

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

/*
 * Sets zero velocities to the arm and halts it.   No destroying happens here
 */
static void haltArm(somatic_d_t &daemon_cx, somatic_motor_t &arm) {
	double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &arm, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
}

/* ********************************************************************************************* */
static void initIMU(somatic_d_t& daemon_cx, ach_channel_t &imu_chan) {
	somatic_d_channel_open(&daemon_cx, &imu_chan, "imu-data", NULL);
}

