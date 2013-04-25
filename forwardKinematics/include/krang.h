#pragma once

#include <amino.h>
#include <somatic.h>
#include <reflex.h>
#include <somatic/daemon.h>

#define SITTING_ANGLE -1.6
#define MAX_LIN_VEL 2.0

#define KRANG_I_LEFT 0
#define KRANG_I_RIGHT 1

typedef enum {
	KRANG_ARM_MODE_HALT,
	KRANG_ARM_MODE_JS,
} krang_arm_mode_t;

typedef struct {
	krang_arm_mode_t mode;
	rfx_ctrl_t G;
	rfx_ctrl_ws_lin_k_t K;
} krang_arm_t;

typedef struct {
	krang_arm_t arm[2];
} krang_state_t;

typedef struct {
	double x[6];
	char b[10];
} krang_js_t;

typedef struct {
  krang_state_t X;										///< The state definition
	ach_channel_t state_chan;
	krang_js_t ui;
} krang_cx_t;

extern krang_cx_t krang_cx;
extern somatic_d_t daemon_cx;

void krang_init( krang_cx_t *cx );

