/**
 * @file helpers.h
 * @author Can Erdogan, Munzir Zafar
 * @date Apr 26, 2013
 * @brief The set of helper functions for the kinematics experiments. These include the 
 * initialization function that handles motor, joystick and perception channels.
 */

#pragma once

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>
#include <amino.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

/// Initializes the somatic daemon context, the motors and joystick/state/transform channels
void init(somatic_d_t& daemon_cx, somatic_motor_t& llwa, somatic_motor_t& rlwa, 
	ach_channel_t& js_chan, ach_channel_t& state_chan, ach_channel_t& chan_transform);

/// Sets the red markers position (x,y,z) if perceived
bool getRedMarkerPosition(somatic_d_t& daemon_cx, ach_channel_t& chan_transform, double* x);

/// Set motor values with joystick input
void setJoystickInput (somatic_d_t& daemon_cx, ach_channel_t& js_chan, somatic_motor_t& llwa, 
	somatic_motor_t& rlwa);
