/*
 * joystick_client.h
 *
 *  Created on: Jul 14, 2013
 *      Author: jscholz
 */

#ifndef JOYSTICK_CLIENT_H_
#define JOYSTICK_CLIENT_H_


#include <ach.h>
#include <somatic.h>
#include <somatic.pb-c.h>
#include <Eigen/Dense>
#include <math/UtilsRotation.h>

#include "util.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

// Channel variables
uint8_t *achbuf_joystick;
size_t n_achbuf_joystick = 1024;
ach_channel_t spacenav_chan;
ach_channel_t spacenav_chan2;

VectorXi getSpacenavButtons(ach_channel_t &joy_chan = spacenav_chan) {

	VectorXi buttons(2); buttons.setZero(2);

	// get spacenav data
	int r = 0;
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
			&protobuf_c_system_allocator,
			4096, &joy_chan );
//cout << ach_result_to_string(static_cast<ach_status_t>(r)) << endl;
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r || ACH_STALE_FRAMES == r) || (js_msg == NULL)) return buttons;

	// pack output vector
//	cout << "button0 raw: " << js_msg->buttons->data[0] << endl;
//	cout << "button1 raw: " << js_msg->buttons->data[1] << endl;
	buttons << js_msg->buttons->data[0], js_msg->buttons->data[1];

	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);

	return buttons;
}

Vector6d getSpacenavConfig(ach_channel_t &chan = spacenav_chan) {

	Vector6d config(6); config.setZero();

	// get joystick data
	int r = 0;
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
			&protobuf_c_system_allocator,
			4096, &chan );

	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return config;

	// extract translation
	Vector3d pos(3);
	for (int j=0; j < 3; j++) pos[j] = js_msg->axes[0].data[j];

	// convert quat to rotation matrix
	Vector3d rotV(3);
	for (int j=0; j < 3; j++) rotV[j] = js_msg->axes[0].data[j+3];

	// pack into config (spacenav has weird frame when read as a joystick)
	config << -pos(1), -pos(0), -pos(2), -rotV(1), -rotV(0), -rotV(2);

	// Free the liberty message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);

	return config;
}

/*
 * Return the joystick pose in the world frame
 *TODO: rename/refactor for spacenav and joy versions
 */
bool getJoystickPose(MatrixXd &pose, Vector6d* config = NULL, ach_channel_t &chan = spacenav_chan) {
	
	Vector6d spnconfig = getSpacenavConfig(chan);
	pose = eulerToTransform(spnconfig, math::XYZ);

	if (config != NULL)
		*config = spnconfig;
}


#endif /* JOYSTICK_CLIENT_H_ */
