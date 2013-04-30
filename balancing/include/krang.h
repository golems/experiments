#ifndef KRANG_H
#define KRANG_H

#include <amino.h>
#include <somatic.h>
#include <reflex.h>
#include <somatic/daemon.h>

#define SITTING_ANGLE -1.6
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
	KRANG_MODE_SIZE
} krang_mode_t;


typedef enum {
	KRANG_ARM_MODE_BAD = 0,
	// valid modes
	KRANG_ARM_MODE_HALT,
	KRANG_ARM_MODE_JS,
	KRANG_ARM_MODE_WS,
	KRANG_ARM_MODE_EXTERNAL,
	// fake mode
	KRANG_ARM_MODE_SIZE
} krang_arm_mode_t;

typedef enum {
	KRANG_EVENT_BAD = 0,
	// valid events
	KRANG_EVENT_START,
	KRANG_EVENT_SIT,
	KRANG_EVENT_STAND,
	KRANG_EVENT_QUIT,
	KRANG_EVENT_STICK,
	KRANG_EVENT_THRESH_SIT_DOWN,
	KRANG_EVENT_THRESH_SIT_UP,
	KRANG_EVENT_THRESH_TORSO_LO,
	KRANG_EVENT_THRESH_TORSO_HI,
	// fake event
	KRANG_EVENT_SIZE
} krang_event_t;

typedef struct {
	// discrete
	krang_mode_t mode;
	
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

	// reference
	double js_lr, js_fb;  // joystick axes
	double imu_ref;	   // imu reference axis

	double q1_ref[2];  // wheel reference pos.
	double dq1_ref[2]; // wheel reference vel.

} krang_state_t;

typedef krang_mode_t
krang_parse_table[KRANG_MODE_SIZE][KRANG_EVENT_SIZE];

typedef struct {
	double x[6];
	char b[10];
} krang_js_t;

typedef struct {

  krang_state_t X;										///< The state definition
	somatic_d_t d_cx;										///< Somatic channel it uses
	krang_parse_table parse_table;			///< The automaton that controls its state changes

	ach_channel_t state_chan;
	ach_channel_t spnav_chan;
	double spnav[6];

	krang_js_t ui;
} krang_cx_t;

typedef struct {
	char str[20];
	double current;
	double last;
	double offset;
} JointPosition;

extern krang_cx_t krang_cx;

const char *krang_mode_str( krang_mode_t mode);
void krang_init( krang_cx_t *cx );
void krang_poll( krang_cx_t *cx );
void krang_parse_init( krang_parse_table tab );
void krang_parse_event( krang_cx_t *cx, krang_event_t ev );
void krang_destroy( krang_cx_t *cx );
void krang_send_state( krang_cx_t *X);
void krang_threshold( krang_cx_t *X);
#endif //KRANG_H
