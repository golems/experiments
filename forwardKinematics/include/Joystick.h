/**
 * @file Joystick.h
 * @author Can Erdogan
 * @date Jan 14, 2013
 * @brief The joystick definitions.
 */

#pragma once 

somatic_d_t daemon_cx;

/// Joystick interface
class Joystick {
public:
	ach_channel_t js_chan;							///< The channel over which the joystick values are read
	krang_js_t jsvals;									///< Contains the button and axes values

public:
	Joystick(const char *);			///< The constructor
	~Joystick() { ach_close(&js_chan); }

	/// Updates the jsvals field with the button and axes info
	void update();											

	/// Control arms with joint/workspace
	static void arm_ctrl(char* b, double* x, double* spnav, krang_state_t *X, double dt);

	/// Process the button, change discrete modes and call appropriate controls
	static void process_input (krang_cx_t *cx, double dt );
};
