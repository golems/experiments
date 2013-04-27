/**
 * @file motor.cpp
 * @author Kasemsit Teeyapan, Can Erdogan
 * @date Aug 15, 2010
 * @brief The interface to talk with the motors.
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include <amino.h>
#include <schkin.h>
#include "krang.h"
#include "Motor.h"

/* ********************************************************************************************** */
Motor::Motor(const char *cmd_chan_name, const char *state_chan_name, 
		const std::string& motor_label, const unsigned int& numModules) {

	// Initialize the motor channel
	somatic_motor_init( &krang_cx.d_cx, &this->motor, numModules, cmd_chan_name, state_chan_name );

	// Set the min and maximum valid position and velocity values
	aa_fset( this->motor.pos_valid_min, -1024.1, numModules);
	aa_fset( this->motor.pos_valid_max, 1024.1, numModules);
	aa_fset( this->motor.vel_valid_min, -1024.1, numModules);
	aa_fset( this->motor.vel_valid_max, 1024.1, numModules);

	// Set the min and maximum position and velocity limits
	aa_fset( this->motor.pos_limit_min, -1024.1, numModules);
	aa_fset( this->motor.pos_limit_max, 1024.1, numModules);
	aa_fset( this->motor.vel_limit_min, -1024.1, numModules);
	aa_fset( this->motor.vel_limit_max, 1024.1, numModules);

	// Set the motor label and the number of modules
	this->label = motor_label;
	this->n_data = numModules;

	// Get readings
	usleep(1e5);
	this->update();
}

/* ********************************************************************************************** */
Motor::~Motor() {
	somatic_motor_destroy( &krang_cx.d_cx, &this->motor );
}

/* ********************************************************************************************** */
void Motor::update() {
	somatic_motor_update( &krang_cx.d_cx, &this->motor );
}

/* ********************************************************************************************** */
void Motor::generate_motorcmd( double *vals, Somatic__MotorParam param ) {
	somatic_motor_cmd( &krang_cx.d_cx, &this->motor, param, vals, this->n_data, NULL);
}

/* ********************************************************************************************** */
// This function is specifically aded to control the digital output pins on the AMC drives, one of 
// which is now being used to override the BMS control of DISCHARGE relay
// PortNumber: 0-15 for virtual output port#1-16 on left wheel servo, 16-19 for right wheel servo
// value can be either 0 or 1
void Motor::digital_out( size_t portNumber, bool value ) {
	somatic_motor_digital_out( &krang_cx.d_cx, &this->motor, portNumber, value);
}

/* ********************************************************************************************** */
void Motor::halt() {

	// Inform the user
	std::cerr << ">> Motor: " << label << " halted\n";

	// Set all the commands to 0. 
	double *command = new double[n_data];
	for (size_t i = 0; i < n_data; i++) command[i] = 0.0;

	// Generate the motor command and send it via somatic. What roles does the flag play
  // as opposed to the command values?
	this->generate_motorcmd(command, SOMATIC__MOTOR_PARAM__MOTOR_HALT);
}

/* ********************************************************************************************** */
void Motor::reset() {

	// Inform the user
	std::cerr << ">> Motor: " << label << " reset\n";

	// Set all the commands to 0. 
	double *command = new double[n_data];
	for (size_t i = 0; i < n_data; i++) command[i] = 0.0;

	// Send the command and wait a bit. 
	this->generate_motorcmd(command, SOMATIC__MOTOR_PARAM__MOTOR_RESET);
	usleep(50000);
}

/* ********************************************************************************************** */
void Motor::set_current(double *target_current) {
	this->generate_motorcmd(target_current, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT);
}

/* ********************************************************************************************** */
void Motor::set_velocity(double *target_vel) {
	this->generate_motorcmd(target_vel, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY);
}

/* ********************************************************************************************** */
void Motor::set_position(double *target_pos) {
	this->generate_motorcmd(target_pos, SOMATIC__MOTOR_PARAM__MOTOR_POSITION);
}
