#include "Controllers.h"
#include <schkin.h>
#include "Dynamics.h"
#include <Eigen/Dense>
/* ********************************************************************************************* */
void krang_threshold( krang_cx_t *cx) {

	// Get the imu and torso values
	static double epsilon = .001;
	double imu = cx->X.q2;  
	double torso = cx->X.q3;  

	// Determine if the robot is in the sit down/up mode
	if(imu < (SITTING_ANGLE - epsilon)) krang_parse_event(cx, KRANG_EVENT_THRESH_SIT_DOWN);
	else if(imu > (SITTING_ANGLE + epsilon)) krang_parse_event(cx, KRANG_EVENT_THRESH_SIT_UP);

	// Determine if the robot is in the torso low/high mode
	if(torso > (2.1 + epsilon)) krang_parse_event(cx, KRANG_EVENT_THRESH_TORSO_LO);
	else if(torso < (2.1 - epsilon)) krang_parse_event(cx, KRANG_EVENT_THRESH_TORSO_HI);
}

#define RAD2DEG(x) (x/M_PI*180.0)

/* ********************************************************************************************* */
void Controllers::balance( double *U, krang_state_t *X) {

	// Set the gains for the imu (th) and the wheel (wh) controllers
	static const double KP_TH	= 745.0;
	static const double KD_TH   = 149.0;
	static const double KP_WH	 = 3.0;				
	static const double KD_WH	 = 10;				
	static const double KD_WH_LR = 15.0;			// The left/right turning gain

	// Express the error terms for the wheels
	// NOTE We take the mean of the reference values (pos/vel) for the two wheels.
	double pref = (X->q1_ref[0] + X->q1_ref[1]) / 2.0;
	double error_wh = X->q1 - pref;
	double vref = (X->dq1_ref[0] + X->dq1_ref[1]) / 2.0;
	double derror_wh = X->dq1 - vref;
	bool debug = 1;
	static int debugCount = 0;
	debug &= (++debugCount % 500 == 1);

	// Create vectors for right and left arms to pass to the com function
	Eigen::VectorXd lq(7), rq(7);
	// ===============================================================
	// Get the current CoM, print the error

	// Print the waist and imu values
	if(debug) printf("\nimu: %.3lf, waist: %.3lf\n", RAD2DEG(X->q2), RAD2DEG(X->q3));

	// Get the current com
	static Dynamics dynamics (TOP_LEVEL_PATH"/data/MassProp.table", true);
	Eigen::Vector3d com = dynamics.com(X->q2, X->q3, 0.0, lq, rq);
	if(debug) printf("com: %.3lf, %.3lf, %.3lf\n", com(0), com(1), com(2));

	// Compute the error term
	double error_th = atan2(com(0), com(1));// + 4*M_PI/180.0;
	if(debug) printf("error: %.3lf\n", RAD2DEG(error_th));	
	double derror_th = X->dq2;

	// Compute the torque for balancing and forward/backward motion
	double u = ((KP_TH * error_th) + (KD_TH * derror_th)) + ((KD_WH *derror_wh) + (KP_WH * error_wh));

	// Set the offsets to enable the left/right motion input from the joystick
	double offset = KD_WH_LR * X->js_lr;
	U[0] = u + offset;
	U[1] = u - offset;
	if(debug) { printf("amc_current: (%.5lf, %.5lf)\n", U[0], U[1]); }
	if(debug) { printf("lq :"); for(int i=0; i<7; i++) {printf("  %.2lf", lq(i)*180.0/M_PI);} printf("\n"); }
	if(debug) { printf("rq :"); for(int i=0; i<7; i++) {printf("  %.2lf", rq(i)*180.0/M_PI);} printf("\n"); }
	if(debug) { fflush(stdout);}
}

/* ********************************************************************************************* */
void Controllers::tosit(double *U, krang_state_t *X) {
	
	// Set the reference positions to current positions and velocities to 0; essentially making
	// the wheels stop.
	// NOTE Even though ref values are not used, they are used by the controller in the next mode.
	krang_cx.X.q1_ref[0] = krang_cx.X.q1_0;
	krang_cx.X.q1_ref[1] = krang_cx.X.q1_1;
	krang_cx.X.dq1_ref[0] = 0;
	krang_cx.X.dq1_ref[1] = 0;

	// The reference angle is the SITTING_ANGLE and the variable is the IMU reading. Perform simple
	// PD control and set the wheel torques.
	double K_TH	= 240;
	double K_DTH   = 60;
	U[0] = K_TH*(X->q2 - SITTING_ANGLE) + K_DTH * X->dq2;
	U[1] = U[0];
}

/* ********************************************************************************************* */
void Controllers::insit (double *U, krang_state_t *X) {

	// Set the gains for position and velocity control
	double Kp = 3;
	double Kv = 20;

	// Set the desired torques using reference position and velocities
	// TODO Decouple the real error term. Basically expand the following and simplify.
	// Ask cerdogan and munzir.
	U[0] = -Kv*(X->dq1_0 - X->dq1_ref[0] + Kp*(X->q1_0 - X->q1_ref[0]));
	U[1] = -Kv*(X->dq1_1 - X->dq1_ref[1] + Kp*(X->q1_1 - X->q1_ref[1]) );
}
