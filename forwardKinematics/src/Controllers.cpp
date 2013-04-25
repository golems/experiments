#include "Controllers.h"
#include <schkin.h>
#include <Eigen/Dense>
#define RAD2DEG(x) (x/M_PI*180.0)

/* ********************************************************************************************* */
void Controllers::arm(krang_cx_t *cx, double dt, double *UR, double *UL) {

	// Set the joint torques based on the discrete arm control mode
	switch(cx->X.arm[KRANG_I_LEFT].mode) {

		case KRANG_ARM_MODE_HALT:															// If in halt mode, don't do anything
			aa_fzero( cx->X.arm[KRANG_I_LEFT].G.dq_r, 7);

		case KRANG_ARM_MODE_JS: {															// Set the joint values directly

			// Make the reference position/orientation to what is now to avoid jerk if we leave this mode
			aa_fcpy(cx->X.arm[KRANG_I_LEFT].G.x_r, cx->X.arm[KRANG_I_LEFT].G.x, 3);
			aa_fcpy(cx->X.arm[KRANG_I_LEFT].G.r_r, cx->X.arm[KRANG_I_LEFT].G.r, 4);

			// Send the torque information (which is most probably set by the joystick)
			aa_fcpy(UL, cx->X.arm[KRANG_I_LEFT].G.dq_r, cx->X.arm[KRANG_I_LEFT].G.n_q);

			// Mirror some of the joints.
			// NOTE This should be done in Joystick class!
			UL[0] *= -1;
			UL[2] *= -1;
			UL[4] *= -1;
			UL[5] *= -1;
		}	break;

		default: break;
	}

	// Right arm
	switch(cx->X.arm[KRANG_I_RIGHT].mode) {
	case KRANG_ARM_MODE_HALT:
		aa_fzero(cx->X.arm[KRANG_I_RIGHT].G.dq_r, 7 );
	case KRANG_ARM_MODE_JS: {
		aa_fcpy( cx->X.arm[KRANG_I_RIGHT].G.x_r, cx->X.arm[KRANG_I_RIGHT].G.x, 3 );
		aa_fcpy( cx->X.arm[KRANG_I_RIGHT].G.r_r, cx->X.arm[KRANG_I_RIGHT].G.r, 4 );
		aa_fcpy( UR, cx->X.arm[KRANG_I_RIGHT].G.dq_r,  cx->X.arm[KRANG_I_RIGHT].G.n_q );
	}	break;
	default: break;
	}
}

