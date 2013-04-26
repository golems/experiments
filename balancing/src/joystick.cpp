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
void Joystick::process_input( krang_cx_t *cx, double dt ) {

	char *b = cx->ui.b;
	double *x = cx->ui.x;
	
	// Set the forward/backward and left/right joystick axes values to zero
	cx->X.js_fb = 0;
	cx->X.js_lr = 0;

	// Check for the start, quit, sit and stand conditions
	if(b[6] && b[7]) krang_parse_event( &krang_cx, KRANG_EVENT_START );
//	else if(b[8]) krang_parse_event( &krang_cx, KRANG_EVENT_QUIT );
	else if(b[9] && !b[5]) krang_parse_event( &krang_cx, KRANG_EVENT_SIT );
	else if(b[9] && b[5]) krang_parse_event( &krang_cx, KRANG_EVENT_STAND );

	// Check for the wheel control - no shoulder button should be pressed
	// None of the shoulder buttons should be pressed
	else if(!b[4] && !b[5] && !b[6] && !b[7]) {
		cx->X.js_fb = -x[1];	// range [-1, 1]
		cx->X.js_lr =  x[2];	// range [-1, 1]
		cx->X.arm[0].mode = KRANG_ARM_MODE_HALT;
		cx->X.arm[1].mode = KRANG_ARM_MODE_HALT;
		wheel_ctrl(&cx->X, dt);
	} 

}
