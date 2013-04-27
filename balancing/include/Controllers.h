/**
 * @file Controllers.h
 * @author Can Erdogan
 * @date Jan 14, 2013
 * @brief The definitions for the various controllers for different modes of operation.
 */

#include "krang-io.h" 

/// The set of controllers
class Controllers {
public:

	/// Computes the end effector position/orientation and the Jacobians
	static void kinematics (krang_state_t *X);

	/// Drives the robot while balancing using the joystick inputs
	static void balance (double *U, krang_state_t *X);

	/// Drives the robot around while in the "insit" mode
	static void insit (double *U, krang_state_t *X);

	/// Makes the robot sit using the imu reading and the reference SITTING_ANGLE
	static void tosit (double *U, krang_state_t *X);

	/// Controls the arm movement
	static void arm (krang_cx_t *cx, double dt, double *UR, double *UL);
};
