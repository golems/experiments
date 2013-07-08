#ifndef KRANG_H
#define KRANG_H

#include <amino.h>
#include <somatic.h>
#include <reflex.h>
#include <somatic/daemon.h>

#define SITTING_ANGLE -1.6
#define STABLE_TH_RANGE 0.785
#define MAX_LIN_VEL 2.0

#define KRANG_I_LEFT 0
#define KRANG_I_RIGHT 1

typedef enum {
	KRANG_MODE_BAD = 0,
	KRANG_MODE_BLOCKED,
	KRANG_MODE_QUIT,
	KRANG_MODE_HALT,
	KRANG_MODE_TOSIT,
	KRANG_MODE_SIT,
	KRANG_MODE_BALANCE,
	KRANG_MODE_TRACK,
	KRANG_MODE_SIZE
} krang_mode_t;

// In KRANG_MODE_TRACK mode, krang will be in one of the following track-modes
typedef enum {
	TRACK_FWD_REV = 0,
	TRACK_LEFT_RIGHT,
	TRACK_CIRCLE
} track_mode_t;

typedef struct {
	// discrete
	krang_mode_t mode;
	track_mode_t track_mode;	

	// continuous
	// filtered states of Krang
	double q1;		  // wheel position [rad]
	double dq1;		 // wheel velocity [rad/s]
	double q2;		  // IMU angle [rad]
	double dq2;		 // IMU velocity [rad/s]
	double q3;		  // Waist angle [rad]
	double dq3;		 // Waist velocity [rad/s]

	double q1_0;		// Left wheel
	double dq1_0;
	double q1_1;		// Right wheel
	double dq1_1;

	double spin;		// Heading direction (+ve when rotation is counter-clockwise from top view)
	double dspin;		// Speed of change of heading direction

	// reference
	double js_lr, js_fb;  // joystick axes
	double imu_ref;	   // imu reference axis

	double q1_ref[2];  // wheel reference pos.
	double dq1_ref[2]; // wheel reference vel.
	
	double spin_ref;
	double dspin_ref;
	
	double u[2]; // control motor input to wheel motors
	
	double pref, vref;
	double x,y;
} krang_state_t;

typedef struct {
  krang_state_t X;										///< The state definition
	somatic_d_t d_cx;										///< Somatic channel it uses
	ach_channel_t state_chan;
} krang_cx_t;

extern krang_cx_t krang_cx;
void krang_threshold( krang_cx_t *X);
#endif //KRANG_H
