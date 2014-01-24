/**
 * @file 01-move.cpp
 * @author Can Erdogan
 * @date May 23, 2013
 * @brief This demo shows the ability to move the grippers with velocity control.
 */

#include <unistd.h>
#include <iostream>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

using namespace std;

somatic_d_t daemon_cx;

int main () {

	// Initialize a daemon context
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "move";
	somatic_d_init(&daemon_cx, &dopt);

	// Create the motor reference for the left gripper
	somatic_motor_t lgripper;
	somatic_motor_init(&daemon_cx, &lgripper, 1, "lgripper-cmd", "lgripper-state");
	cout << "Gripper value: " << lgripper.pos[0] << endl;
	usleep(1e5);

	// Set the min/max values for valid and limits values
	double** limits [] = {
		&lgripper.pos_valid_min, &lgripper.vel_valid_min, 
		&lgripper.pos_limit_min, &lgripper.pos_limit_min, 
		&lgripper.pos_valid_max, &lgripper.vel_valid_max, 
		&lgripper.pos_limit_max, &lgripper.pos_limit_max};
	for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 1);
	for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 1);
	
	// Update and reset them
	somatic_motor_update(&daemon_cx, &lgripper);
	somatic_motor_cmd(&daemon_cx, &lgripper, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);

	// Get the motor value
	somatic_motor_update(&daemon_cx, &lgripper);
	cout << "Gripper value: " << lgripper.pos[0] << endl;

	// Command the gripper to move
	double dq = -0.1;
	somatic_motor_cmd(&daemon_cx, &lgripper, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, &dq, 1, NULL);
	usleep(0.1 * 1e6);

	// Command the gripper to stop
	dq = 0.0;
	somatic_motor_cmd(&daemon_cx, &lgripper, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, &dq, 1, NULL);
}
