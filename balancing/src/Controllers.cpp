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
	for(int i=0; i<7; i++) {
		lq(i) = X->arm[0].G.q[i];
		rq(i) = X->arm[1].G.q[i];
	}
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

	bool debug = 1;
	static int count = 0;
	debug &= (count++ % 500 == 1);
	if(false) {
		printf("q1: (%.5lf, %.5lf), q1_ref: (%.5lf, %.5lf)\n", X->q1_0, X->q1_1, X->q1_ref[0], X->q1_ref[1]);
		printf("dq1: (%.5lf, %.5lf), dq1_ref: (%.5lf, %.5lf)\n", X->dq1_0, X->dq1_1, X->dq1_ref[0], X->dq1_ref[1]);
	}
	
  if(debug==false) return;

	// Print the estimation of com and error to see everything makes sense before starting the balancing
	// Create vectors for right and left arms to pass to the com function
	Eigen::VectorXd lq(7), rq(7);
	for(int i=0; i<7; i++) {
		lq(i) = X->arm[0].G.q[i];
		rq(i) = X->arm[1].G.q[i];
	}
	// Print the waist and imu values
  printf("\nimu: %.3lf, waist: %.3lf\n", RAD2DEG(X->q2), RAD2DEG(X->q3));

	// Get the current com
	static Dynamics dynamics (TOP_LEVEL_PATH"/data/MassProp.table", true);
	Eigen::Vector3d com = dynamics.com(X->q2, X->q3, 0.0, lq, rq);
  printf("com: %.3lf, %.3lf, %.3lf\n", com(0), com(1), com(2));

	// THIS CODE IS USED FOR THE EXPERIMENT
	// Compute the error term
	double error_th = atan2(com(0), com(1));// + 4*M_PI/180.0;
  printf("insit balancing error: %.3lf\n", RAD2DEG(error_th));	
	double derror_th = X->dq2;
	printf("amc_current: (%.5lf, %.5lf)\n", U[0], U[1]);
	printf("lq :"); for(int i=0; i<7; i++) {printf("  %.2lf", lq(i)*180.0/M_PI); } printf("\n");
	printf("rq :"); for(int i=0; i<7; i++) {printf("  %.2lf", rq(i)*180.0/M_PI); } printf("\n");
	fflush(stdout);
}


/* ********************************************************************************************* */
void Controllers::kinematics (krang_state_t *X) {

	// Compute arm kinematics for each arm
	for(size_t i = 0; i < 2; i++) {

		// =============================================================================
		// Prepare the rotation matrix of the base of the arm

		// Create the rotation matrix from the arm to the torso
		double RR[9] = {0,-1,0, 0,0,-1, 1,0,0}; 

		// Get the rotation matrix between the wheel axis to the spine where q2 and q3 are
		// the imu and waist module readings which lead to rotation around y-axis
		double Rth[9];
		aa_tf_yangle2rotmat((X->q2 + X->q3), Rth);

		// Compute the rotation of the base of the arm wrt wheel axis 
		aa_tf_9mul(Rth, RR, X->arm[i].R0);

		// =============================================================================
		// Perform forward kinematics to get the end effector position and orientation
	
		// The end effector rotation with respect to the arm base is identity in zero
		// configuration.	
		double Ree [9] = {1,0,0, 0,1,0, 0,0,1};

		// Get the end-effector position and orientation using forward kinematics
		// NOTE What is the 3rd argument? Why the position of the arm base is not used?
		double R[9];
		double zeroes [3] = {0,0,0};
		schkin_lwa3_fk(X->arm[i].G.q, X->arm[i].R0, zeroes,	Ree, zeroes, R, X->arm[i].G.x);
	   
		// ====================================================================
		// Compute the jacobian and update the reflex structure

		// Set the rotation matrix of the reflex struct after converting it to a quaternion
		aa_tf_rotmat2quat(R, X->arm[i].G.r);

		// Compute the jacobian using the current joint angles	
		schkin_lwa3_jacobian_(X->arm[i].G.q, X->arm[i].R0, zeroes, Ree, zeroes, X->arm[i].G.J);
	}
}

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

		case KRANG_ARM_MODE_WS: {										// Set the joint values using workspace control

			// Compute the joint vels and fill in "UL". It also updates dx_r (?) in reflex structure.
			int r = rfx_ctrl_ws_lin_vfwd(&cx->X.arm[KRANG_I_LEFT].G, &cx->X.arm[KRANG_I_LEFT].K, UL);

			// Change the position reference given the velocities for 'dt' duration
			aa_la_axpy(3, dt, cx->X.arm[KRANG_I_LEFT].G.dx_r, cx->X.arm[KRANG_I_LEFT].G.x_r);

			// Change the rotation reference given the velocities for 'dt' duration. 
			// Note the conversion from the quaternion to the rpy (?) angle representation.
			double xr[3];
			aa_tf_quat2rotvec(cx->X.arm[KRANG_I_LEFT].G.r_r, xr);
			aa_la_axpy(3, dt, cx->X.arm[KRANG_I_LEFT].G.dx_r + 3, xr);
			aa_tf_rotvec2quat(xr, cx->X.arm[KRANG_I_LEFT].G.r_r);

		} break;
	default: break;
	}

	// Right arm
	switch(cx->X.arm[KRANG_I_RIGHT].mode) {
	case KRANG_ARM_MODE_HALT:
		aa_fzero(cx->X.arm[KRANG_I_RIGHT].G.dq_r, 7 );
	case KRANG_ARM_MODE_JS:

		aa_fcpy( cx->X.arm[KRANG_I_RIGHT].G.x_r,
				 cx->X.arm[KRANG_I_RIGHT].G.x, 3 );
		aa_fcpy( cx->X.arm[KRANG_I_RIGHT].G.r_r,
				 cx->X.arm[KRANG_I_RIGHT].G.r, 4 );

		aa_fcpy( UR, cx->X.arm[KRANG_I_RIGHT].G.dq_r,  cx->X.arm[KRANG_I_RIGHT].G.n_q );
		break;
	case KRANG_ARM_MODE_WS:
	{
		/* Squared magnitude of force's applied to force torque sensor ([0,1,2]), ([3,4,5]) are torques */
		//float ft_force_magn = cx->X.arm[KRANG_I_RIGHT].ft[0]*cx->X.arm[KRANG_I_RIGHT].ft[0] + cx->X.arm[KRANG_I_RIGHT].ft[1]*cx->X.arm[KRANG_I_RIGHT].ft[1] + cx->X.arm[KRANG_I_RIGHT].ft[2]*cx->X.arm[KRANG_I_RIGHT].ft[2];
		//float ft_torque_magn = cx->X.arm[KRANG_I_RIGHT].ft[3]*cx->X.arm[KRANG_I_RIGHT].ft[3] + cx->X.arm[KRANG_I_RIGHT].ft[4]*cx->X.arm[KRANG_I_RIGHT].ft[4] + cx->X.arm[KRANG_I_RIGHT].ft[5]*cx->X.arm[KRANG_I_RIGHT].ft[5];
		aa_dump_vec(stdout, cx->X.arm[KRANG_I_RIGHT].ft , 6);

		//double oldKp[6] = {0,0,0,0,0,0}; /* Initial gains are zeroed out */
		//aa_fcpy( oldKp, cx->X.arm[KRANG_I_RIGHT].K.p , 6); /* Backup original Kp gains */

		//printf("FT FORCE & TORQUE MAGNITUDES : %g %g\n", ft_force_magn, ft_torque_magn);
		//aa_dump_vec(stdout, cx->X.arm[KRANG_I_RIGHT].K.p , 6);

		//if (ft_force_magn > 20.0) /* If forces above threshold, ignore positional gains */
		//{
			/* zero out positional gains on right arm */
			//aa_fzero(cx->X.arm[KRANG_I_RIGHT].K.p,3);
		//}

		 int r = rfx_ctrl_ws_lin_vfwd(
			&cx->X.arm[KRANG_I_RIGHT].G,
			&cx->X.arm[KRANG_I_RIGHT].K,
			UR );

		/* Low pass band filter on joint velocities */
		/*for (int i=0; i<7; ++i) {
			if (abs(UR[i]) < 0.0001) {
				UR[i] = 0;
			}
		}*/
		//aa_dump_vec(stdout, UR , 7);
		//aa_dump_vec(stdout, cx->X.arm[KRANG_I_RIGHT].K.p , 6);

		/* Put original positional gains back into K.p */
		//aa_fcpy( cx->X.arm[KRANG_I_RIGHT].K.p , oldKp, 6);
		//aa_dump_vec(stdout, cx->X.arm[KRANG_I_RIGHT].K.p , 6);

		double xr[3];
		aa_tf_quat2rotvec( cx->X.arm[KRANG_I_RIGHT].G.r_r, xr );

		aa_la_axpy(3, dt, cx->X.arm[KRANG_I_RIGHT].G.dx_r,
				   cx->X.arm[KRANG_I_RIGHT].G.x_r );

		aa_la_axpy(3, dt, cx->X.arm[KRANG_I_RIGHT].G.dx_r + 3,
				   xr );
		aa_tf_rotvec2quat( xr, cx->X.arm[KRANG_I_RIGHT].G.r_r );

	}
	break;
	default: break;
	}





}

