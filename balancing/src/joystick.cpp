/**
 * @file joystick.cpp
 * @author Kasemsit Teeyapan, Can Erdogan
 * @date Aug 15, 2010
 * @brief The interface to talk with the joystick.
 */

// TODO: We want to be able to quit the program using button 9 on joystick

#include <somatic.h>
#include <somatic/daemon.h>
#include <amino.h>
#include <schkin.h>
#include "krang.h"
#include "krang-io.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <Eigen/Dense>

using namespace std;

/* ********************************************************************************************** */
// The predefined locations for the left arm. For the right arm the first joint is negated.
const double goal0_zero [7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const double goal0_sleep [7] = {-0.47, -M_PI_2, -M_PI_2, M_PI_2, 0.0, M_PI_2, 0.0};

/* ********************************************************************************************** */
/// Prints the arm position
void printArm (const char* header, const double* val) {
	printf("%s{", header);
	for(size_t i = 0; i < 7; i++) printf("%1.3lf ", val[i]);
	printf("}\n");
	fflush(stdout);
}

/* ********************************************************************************************** */
/// Creates an Eigen::VectorXd of size 7 from the given double array
Eigen::VectorXd armEigen (const double* val) {
	Eigen::VectorXd result (7);
	for(size_t i = 0; i < 7; i++) result(i) = val[i];
	return result;
}

/* ********************************************************************************************** */
/// Prints an Eigen vector in a nice format
void printEigen (const char* header, const Eigen::VectorXd& val) {
	printf("%s{", header);
	for(size_t i = 0; i < val.size(); i++) printf("%1.3lf ", val(i));
	printf("}\n");
	fflush(stdout);
}

/* ********************************************************************************************** */
/// Fixes the reference and output velocity discrepancy problem for the left
/// arm. See docs.
void fixLeftArm (double* val) {
	val[0] = -val[0];
	val[2] = -val[2];
	val[4] = -val[4];
	val[5] = -val[5];
}

/* ********************************************************************************************** */
Joystick::Joystick(const char* chan_name){

	// Open Joystick channel
	int r  = ach_open(&js_chan, chan_name , NULL);
	aa_hard_assert(r == ACH_OK,
				   "Ach failure %s on opening Joystick channel (%s, line %d)\n",
				   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Sleep (why?) and update the values
	usleep(10000);
	this->update();
}

/* ********************************************************************************************** */
void Joystick::update() {

	// Get the joystick message with somatic
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );

	// Make sure the message is received correctly
	somatic_d_check( &krang_cx.d_cx, SOMATIC__EVENT__PRIORITIES__EMERG,
					 SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
					 ACH_OK != r || ACH_STALE_FRAMES != r,
					 NULL,
					 "Ach failure %s on joystick data receive "
					 "(%s, line %d)\n",
					 ach_result_to_string(static_cast<ach_status_t>(r)),
					 __FILE__, __LINE__);

	// Make sure that the ach output is OK.
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

	// Make sure the button data is received correctly
	bool buttonDataGood = somatic_d_check_msg( &krang_cx.d_cx, js_msg->buttons &&
			js_msg->buttons->data && (js_msg->buttons->n_data >= sizeof(jsvals.b)/sizeof(jsvals.b[0])),
 		  "msg_msg", "bad button data");

	// Make sure the axes data is received correctly
	bool axesDataGood = somatic_d_check_msg( &krang_cx.d_cx, js_msg->axes &&
			js_msg->axes->data && (js_msg->axes->n_data >= sizeof(jsvals.x)/sizeof(jsvals.x[0])),
			"msg_msg", "bad axes data");

	// If everything is OK, record the data in local fields
	if(buttonDataGood && axesDataGood) {

		// Copy the button values 
		for(size_t i = 0; i < sizeof(jsvals.b)/sizeof(jsvals.b[0]); i++) {
			jsvals.b[i] = js_msg->buttons->data[i] ? 1 : 0;
		}

		// Copy the axes values
		memcpy(jsvals.x, js_msg->axes->data, sizeof(jsvals.x));

		// Free the joystick message
		somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
	}
}

/* ********************************************************************************************** */
void Joystick::wheel_ctrl(krang_state_t *X, double dt) {

    // Velocity control when sitting
    X->dq1_ref[0] = MAX_LIN_VEL*X->js_fb;
    X->dq1_ref[1] = X->dq1_ref[0];

    // Moving left/right: Velocity control for heading
    double max_ang_vel = 2.0;
    X->dq1_ref[0] += max_ang_vel*X->js_lr;
    X->dq1_ref[1] -= max_ang_vel*X->js_lr;

    // integrate
    X->q1_ref[0] += dt * X->dq1_ref[0];
    X->q1_ref[1] += dt * X->dq1_ref[1];
}

/* ********************************************************************************************** */
/// Makes the arms reach a goal
void Joystick::arm_goal(char* b, double* x, double* spnav, krang_state_t *X, double dt, bool left) {

	static int c_ = 0;
	c_++;
	bool debug = 1;
	debug &= (c_ % 100 == 1);

	if(debug) printf("\narm_goal for arm %s\n", left ? "left" : "right");
	size_t armIndex1 = left ? 0 : 1;
	size_t armIndex2 = left ? KRANG_I_LEFT : KRANG_I_RIGHT;

	// Set the gains
	static const double armKP = 1.0;
	static const double armKD = 0.1;
	static const double kThreshold = 0.3;

	// Determine the goal state
	Eigen::VectorXd goal;
	if(b[0]) goal = armEigen(goal0_zero);
	else if(b[1]) goal = armEigen(goal0_sleep);
	else return;

	// If this is the right arm, negate the first joint
	if(left) goal = -goal;
	if(debug) printEigen("goal: ", goal);

	// Determine the state
	Eigen::VectorXd pos = armEigen(X->arm[armIndex1].G.q);
	Eigen::VectorXd vel = armEigen(X->arm[armIndex1].G.dq);
	
	// Compute the error
	Eigen::VectorXd pos_error = pos - goal;
	Eigen::VectorXd vel_error = vel;
	if(debug) printEigen("pos: ", pos);
	if(debug) printEigen("pos_error: ", pos_error);

	// Compute the input based on the errors
	Eigen::VectorXd input (7);
	input = - (armKP * pos_error + armKD * vel_error);
	
	// Threshold the input
	for(size_t i = 0; i < 7; i++) {
		if(input(i) > kThreshold) input(i) = kThreshold;
		else if(input(i) < -kThreshold) input(i) = -kThreshold;
	}	

	// Fix the left arm discrepancy problem
	double input_arr [7];
	for(size_t i = 0; i < 7; i++) input_arr[i] = input(i);
	if(left) fixLeftArm(input_arr);
	if(debug) printArm("input: ", input_arr);
	
	// Set the input velocity in the reflex structure
	memcpy(&X->arm[armIndex2].G.dq_r[0], input_arr, 7*sizeof(double));

	// Set the arm mode
	// TODO Another mode should exist for this purpose
	X->arm[armIndex2].mode = KRANG_ARM_MODE_JS;

}

/* ********************************************************************************************** */
void Joystick::arm_ctrl(char* b, double* x, double* spnav, krang_state_t *X, double dt) {

	static int c_ = 0;
	c_++;
	bool debug = 0;
	debug &= (c_ % 100 == 1);

	if(debug) {
		printArm("\nLeft arm: ", (X->arm[0].G.q));
		printArm("\tvel: ", (X->arm[0].G.dq));
		printArm("Right arm: ", (X->arm[1].G.q));
		printArm("\tvel: ", (X->arm[1].G.dq));
	}

	// ============================================================================
	// Running left arm: Pressing only top or bottom left shoulder is going 
	// to move only wrist or shoulder joints respectively. If both are 
	// pressed, then workspace control.
	// NOTE: In the following, we set the discrete mode and the continuous values.

	// Control left shoulder/elbow mode
	if (!b[4] && b[6]) {							
		memcpy(X->arm[KRANG_I_LEFT].G.dq_r, x, 4*sizeof(double));
		if(debug) printArm("Ref. velocity: ", X->arm[KRANG_I_LEFT].G.dq_r); 
		X->arm[KRANG_I_LEFT].mode = KRANG_ARM_MODE_JS;
	} 

	// Move the arm to a predefined position with a PD controller
	else if (b[4] && (b[0] || b[1] || b[2] || b[3])) {
		arm_goal(b, x, spnav, X, dt, true);
	}

	// Control left wrist mode
	else if (b[4] && !b[6]) {																
		memcpy(&X->arm[KRANG_I_LEFT].G.dq_r[4], x, 3*sizeof(double));
		if(debug) printArm("Ref. velocity: ", X->arm[KRANG_I_LEFT].G.dq_r); 
		X->arm[KRANG_I_LEFT].mode = KRANG_ARM_MODE_JS;
	} 

	// Control using workspace control
	else if (b[4] && b[6]) {

		// Set the workspace left control mode
		X->arm[KRANG_I_LEFT].mode = KRANG_ARM_MODE_WS;

		// Put spnav axes in krang frame
		double R[9]; 
		aa_tf_zangle2rotmat( -M_PI_2, R );
		aa_tf_9(R, spnav,  X->arm[KRANG_I_LEFT].G.dx_r );
		aa_tf_9(R, spnav+3, X->arm[KRANG_I_LEFT].G.dx_r+3 );

		// Scale down the forces
		aa_fcpy(X->arm[KRANG_I_LEFT].G.dx_r, x, 6);
		aa_la_scal(3, .25, X->arm[KRANG_I_LEFT].G.dx_r );
		aa_la_scal(3, .50, X->arm[KRANG_I_LEFT].G.dx_r+3 );
	} 

	// Disable the arm movement
	else {
		X->arm[KRANG_I_LEFT].mode = KRANG_ARM_MODE_HALT;
	}

	// ============================================================================
	// Running right arm: the same story as left arm.

	// Control right shoulder/elbow mode
	if (!b[5] && b[7]) {
		memcpy(X->arm[KRANG_I_RIGHT].G.dq_r, x, 4*sizeof(double));
		if(debug) printArm("Ref. velocity: ", X->arm[KRANG_I_RIGHT].G.dq_r); 
		X->arm[KRANG_I_RIGHT].mode = KRANG_ARM_MODE_JS;
	} 

	// Move the arm to a predefined position with a PD controller
	else if (b[5] && (b[0] || b[1] || b[2] || b[3])) {
		arm_goal(b, x, spnav, X, dt, false);
	}

	// Control right wrist mode
	else if (b[5] && !b[7]) {
		memcpy(&X->arm[KRANG_I_RIGHT].G.dq_r[4], x, 3*sizeof(double));
		if(debug) printArm("Ref. velocity: ", X->arm[KRANG_I_RIGHT].G.dq_r); 
		X->arm[KRANG_I_RIGHT].mode = KRANG_ARM_MODE_JS;
	} 
	
	// Control using workspace control
	else if (b[5] && b[7]) {

		// Set the workspace right control mode
		X->arm[KRANG_I_RIGHT].mode = KRANG_ARM_MODE_WS;

		// Put spnav axes in krang frame
		double R[9]; 
		aa_tf_zangle2rotmat( -M_PI_2, R );
		aa_tf_9(R, spnav, X->arm[KRANG_I_RIGHT].G.dx_r );
		aa_tf_9(R, spnav+3, X->arm[KRANG_I_RIGHT].G.dx_r+3 );

		// Set the joystick values to the reflex library struct for it
		// to compute the right joint values in workspace control
		X->arm[KRANG_I_RIGHT].G.dx_r[0] = -x[1];
		X->arm[KRANG_I_RIGHT].G.dx_r[1] = -x[2];
		X->arm[KRANG_I_RIGHT].G.dx_r[2] = -x[3];
		X->arm[KRANG_I_RIGHT].G.dx_r[3] =  x[0];
		X->arm[KRANG_I_RIGHT].G.dx_r[4] =  x[4];
		X->arm[KRANG_I_RIGHT].G.dx_r[5] =  x[5];

		// Scale down the forces
		aa_la_scal(3, .25, X->arm[KRANG_I_RIGHT].G.dx_r );
		aa_la_scal(3, .50, X->arm[KRANG_I_RIGHT].G.dx_r+3 );
	} 

	// Disable the arm movement
	else {
		X->arm[KRANG_I_RIGHT].mode = KRANG_ARM_MODE_HALT;
	}
}

/* ********************************************************************************************** */
void Joystick::process_input( krang_cx_t *cx, double dt ) {

	bool debug = 0;
	static int counter = 0;
	if(counter == 0) {
		printf(
			">> Press buttons [7]&[8] to enable wheels.\n"
			">> Press button  [9] to quit.\n"
			">> Press button  {1,2,3,4} to quit.\n"
			">> Press button  [6]&[10] to sit.\n"
			">> Hold  buttons [5]|[6] for wrist jointspaces\n"
			">> Hold  buttons [7]|[8] for shoulder jointspaces\n"
			);
	}
	counter++;
	debug &= (counter % 500 == 1);

	char *b = cx->ui.b;
	double *x = cx->ui.x;
	
	if(debug) {
		printf("buttons: ");
		for(size_t i = 0; i < 10; i++) 
			printf("%c\t", (b[i] == 0) ? '0' : '1');
		printf("\naxes: ");
		for(size_t i = 0; i < 6; i++) 
			printf("%.2lf\t", x[i]);
		printf("\n");
		fflush(stdout);
	}
	
	// Set the forward/backward and left/right joystick axes values to zero
	cx->X.js_fb = 0;
	cx->X.js_lr = 0;

	// Set the reference velocities of the arms to zero to begin with
	aa_fzero(cx->X.arm[0].G.dq_r, cx->X.arm[0].G.n_q);
	aa_fzero(cx->X.arm[1].G.dq_r, cx->X.arm[1].G.n_q);

	// Check for the start, quit, sit and stand conditions
	if(b[6] && b[7]) krang_parse_event( &krang_cx, KRANG_EVENT_START );
//	else if(b[8]) krang_parse_event( &krang_cx, KRANG_EVENT_QUIT );
	else if(b[9] && !b[5]) krang_parse_event( &krang_cx, KRANG_EVENT_SIT );
	else if(b[9] && b[5]) krang_parse_event( &krang_cx, KRANG_EVENT_STAND );

	// Check for the wheel control - no shoulder button should be pressed
	// None of the shoulder buttons should be pressed
	else if(!b[4] && !b[5] && !b[6] && !b[7]) {
		if(debug) printf("wheel control ...\n");
		cx->X.js_fb = -x[1];	// range [-1, 1]
		cx->X.js_lr =  x[2];	// range [-1, 1]
		cx->X.arm[0].mode = KRANG_ARM_MODE_HALT;
		cx->X.arm[1].mode = KRANG_ARM_MODE_HALT;
		wheel_ctrl(&cx->X, dt);
	} 

	// Check for arm ctrl
	else {
		if(debug) printf("arm control ...\n");
		arm_ctrl(b, x, &(cx->spnav[0]), &cx->X, dt);
	}

	// Check for the waist control
	if(x[5]<-0.9) {
		cx->X.waist_mode = KRANG_WAIST_MODE_TILT_FORWARD;
	}
	else if(x[5]>0.9) {
		cx->X.waist_mode = KRANG_WAIST_MODE_TILT_BACKWARD;
	}
	else {
		cx->X.waist_mode = KRANG_WAIST_MODE_HALT;
	}
}
