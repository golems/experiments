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
#include <iostream>
#include "Joystick.h"
#include <iomanip>
#include <stdio.h>
#include <Eigen/Dense>
#include <unistd.h>

using namespace std;

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
void Joystick::arm_ctrl(char* b, double* x, double* spnav, krang_state_t *X, double dt) {

	static int c_ = 0;
	c_++;


	// ============================================================================
	// Running left arm: Pressing only top or bottom left shoulder is going 
	// to move only wrist or shoulder joints respectively. If both are 
	// pressed, then workspace control.
	// NOTE: In the following, we set the discrete mode and the continuous values.

	// Control left shoulder/elbow mode
	if (!b[4] && b[6]) {							
		memcpy(X->arm[KRANG_I_LEFT].G.dq_r, x, 4*sizeof(double));
		X->arm[KRANG_I_LEFT].mode = KRANG_ARM_MODE_JS;
	} 

	// Control left wrist mode
	else if (b[4] && !b[6]) {																
		memcpy(&X->arm[KRANG_I_LEFT].G.dq_r[4], x, 3*sizeof(double));
		X->arm[KRANG_I_LEFT].mode = KRANG_ARM_MODE_JS;
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
		X->arm[KRANG_I_RIGHT].mode = KRANG_ARM_MODE_JS;
	} 

	// Control right wrist mode
	else if (b[5] && !b[7]) {
		memcpy(&X->arm[KRANG_I_RIGHT].G.dq_r[4], x, 3*sizeof(double));
		X->arm[KRANG_I_RIGHT].mode = KRANG_ARM_MODE_JS;
	} 
	
	// Disable the arm movement
	else {
		X->arm[KRANG_I_RIGHT].mode = KRANG_ARM_MODE_HALT;
	}
}

/* ********************************************************************************************** */
void Joystick::process_input( krang_cx_t *cx, double dt ) {

	// Set the reference velocities of the arms to zero to begin with
	aa_fzero(cx->X.arm[0].G.dq_r, cx->X.arm[0].G.n_q);
	aa_fzero(cx->X.arm[1].G.dq_r, cx->X.arm[1].G.n_q);

	// Set the input for arms
	arm_ctrl(cx->ui.b, cx->ui.x, NULL, &cx->X, dt);
}
