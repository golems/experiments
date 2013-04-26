/* -*- mode: C; c-basic-offset: 4; indent-tabs-mode: nil  -*- */
/*
 * krang_io.h
 *
 *  Created on: Aug 15, 2010
 *      Author: kasemsit
 */

#ifndef KRANG_IO_H_
#define KRANG_IO_H_

#include <unistd.h>
#include <string>
#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <ach.h>
#include <somatic.h>
#include <somatic/util.h>
#include <somatic/daemon.h>
#include <somatic/motor.h>
#include <somatic.pb-c.h>
#include <imud.h>
#include <pciod.h>
#include <filter.h>
#include <krang.h>

#include "Joystick.h"
#include "Imu.h"
#include "Motor.h"

// Gravity
static const double g	= 9.80665;

// Wheel parameters
static const double L1_m = 4.4 + 4.4;
static const double L1_I = ((3/4)*0.05*0.05 + 0.23*0.23)*L1_m + 0.5*L1_m*0.076*0.076;
static const double wheel_radius = 0.263;
static const double wheel_dist = 0.71;

// Body parameters
static const double L2_m = 76.8;
static const double L2_len = 0.507;
static const double L2_com_len = .3;
static const double L2_I = (1/12)*L2_m*(0.25*0.25+0.5*0.5);
static const double L2_com_rot = -0.258;

// Torso parameters
// NOTE: If spine is perfectly vertical, q2 = q3_offset + q3
static const double L3_m= 14;
static const double L3_com_len = 0.4064;
static const double L3_I	= (1/12)*L3_m*L3_com_len*L3_com_len;  
static const double L3_com_rot = 0.0335;
static const double L3_len = .7366;
static const double Q3_offset = .296; 

// Arm and gripper parameters
static const double L4_m			= 17.277;
static const double L5_m			= 1.6;

// Offsets (?)
//static const double imu_offset	  = 0.0; //0.1843;
static const double shoulder_offset = 0.153495;

/**
 *	  \brief Collection of Kalman-filtered states of the robot with utilities to compute the center of mass of the robot.
 */
struct Krang {

/*
	// filtered states of Krang
	double q1;		  // wheel position [rad]
	double dq1;		 // wheel velocity [rad/s]
	double q2;		  // IMU angle [rad]
	double dq2;		 // IMU velocity [rad/s]
	double q3;		  // Torso angle [rad]
	double dq3;		 // Torso velocity [rad/s]

	double q1_0;		// Left wheel
	double dq1_0;
	double q1_1;		// Right wheel
	double dq1_1;

	double imu_balancing_angle;  // balancing angle that IMU has to be in order to balance (rad)
*/
	// Kalman filter
	krang_state_t* state;
	filter_kalman_t *kf;
	void krang_kalman_filter(/*double *, double **/);
	JointPosition position[17];
	char positionOffsetsPath[100];
	char lastPositionsPath[100];
public:

	Imu *imu;
	Joystick *js;
	Motor *amc, *waist, *torso, *rlwa, *llwa;

	Krang(krang_state_t* state);
	~Krang();
	void update();
	void get_state(krang_state_t *X);
	void get_js(krang_js_t *js);
	void update_and_filter();
	static void krang_dump_state(Krang *krang, double dt);
	void read_position_offsets_from_file();
	void read_previous_positions_from_file();
	void set_position_offsets();
	void set_velocities_to_zero();
	void populate_positions();
	void write_position_offsets_to_file();
	void write_positions_to_file();
	
// Utilities
	void get_arm_cm(double *cm_xyz);	/// Compute arm C.M. w.r.t. to shoulder joint

};

#endif /* KRANG_IO_H_ */
