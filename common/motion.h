/**
 * @file motion.h
 * @author Can Erdogan
 * @date June 12, 2013
 * @brief This file includes common motion operations such as moving the arms with the joystick, 
 * manipulating the grippers and etc.
 */

#pragma once

#include <iostream>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

/* ********************************************************************************************* */
/// Sets motor values with joystick input
static void setJoystickInput (somatic_d_t& daemon_cx, ach_channel_t& js_chan, somatic_motor_t& llwa, 
		somatic_motor_t& rlwa) {

	static const bool debug = 0;

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

	// Get the values
	char b [10];
	double x [6];
	for(size_t i = 0; i < 10; i++) 
		b[i] = js_msg->buttons->data[i] ? 1 : 0;
	memcpy(x, js_msg->axes->data, sizeof(x));
	
	// Scale down the x values
	double scaleDownFactor = 0.5;
	for(size_t i = 0; i < 6; i++) x[i] *= scaleDownFactor;

	// Print the data for debugging purposes
	if(debug) {
		std::cout << "\nx: ";
		for(size_t i = 0; i < 6; i++) std::cout << x[i] << ", ";
		std::cout << "\nb: ";
		for(size_t i = 0; i < 10; i++) std::cout << "'" << (b[i] ? '1' : '0') << "', ";
	}

	// Check the buttons for each arm and apply velocities accordingly
	// For left: 4 or 6, for right: 5 or 7, lower arm button is smaller (4 or 5)
	somatic_motor_t* arm [] = {&llwa, &rlwa};
	for(size_t arm_idx = 0; arm_idx < 2; arm_idx++) {

		// Initialize the input
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Change the input based on the lower or higher button input
		bool inputSet = true;
		size_t lowerButton = 4 + arm_idx, higherButton = 6 + arm_idx;
		if(b[lowerButton] && !b[higherButton]) memcpy(&dq[4], x, 3*sizeof(double));
		else if(!b[lowerButton] && b[higherButton]) memcpy(dq, x, 4*sizeof(double));
		else inputSet = false;
		
		// Set the input for this arm
		if(inputSet)
			somatic_motor_cmd(&daemon_cx, arm[arm_idx], SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dq, 7, 
				NULL);
	}
	
	// Free the joystick message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
}
