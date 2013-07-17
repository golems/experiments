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

MatrixXd T_spacenav = (MatrixXd(4,4) <<  0,-1,0,0, -1,0,0,0, 0,0,-1,0, 0,0,0,1).finished();

// use when converting matrices for visualization or control.  anything dart related
static math::RotationOrder dartRotOrder = math::XYZ;

// use when converting from spacenav
static math::RotationOrder spnavRotOrder = math::XYZ;

VectorXi getSpacenavButtons(ach_channel_t &joy_chan = spacenav_chan) {

	VectorXi buttons(2); buttons.Zero(2);

	// get spacenav data
	int r = 0;
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
			&protobuf_c_system_allocator,
			4096, &joy_chan );
	cout << ach_result_to_string(static_cast<ach_status_t>(r)) << endl;
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r || ACH_STALE_FRAMES == r) || (js_msg == NULL)) return buttons;

	// pack output vector
	cout << "button0 raw: " << js_msg->buttons->data[0] << endl;
	cout << "button1 raw: " << js_msg->buttons->data[1] << endl;
	buttons << js_msg->buttons->data[0], js_msg->buttons->data[1];

	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);

	return buttons;
}

Vector6d getSpacenavConfig(ach_channel_t &chan = spacenav_chan) {

	Vector6d config(6); config.Zero();

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

	// pack into config
	config << -pos(1), -pos(0), -pos(2), -rotV(1), -rotV(0), -rotV(2); // correct raw (and control) (this is XYZ)
	//config << -pos(1), -pos(0), -pos(2), -rotV(2), -rotV(0), -rotV(1); // works for grip visualization (this is ZYX)

	/*
	 * positive rotation is CW with positive axis facing AWAY from you
	 */

	// Free the liberty message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);

	return config;
}

/*
 * Return the joystick pose in the world frame
 *TODO: rename/refactor for spacenav and joy versions
 */
bool getJoystickPose(MatrixXd &pose, Vector6d* config = NULL, ach_channel_t &chan = spacenav_chan) {
//	// get joystick data
//	int r = 0;
//	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
//			&protobuf_c_system_allocator,
//			4096, &joystick_chan );
//
//	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return -1;
//
//	// scale dimensions
//	double weights[] = {1, 1, 1, 1, 1, 1};
//
//	// extract translation
//	Vector3d pos(3);
//	for (int j=0; j < 3; j++)
//		pos[j] = js_msg->axes[0].data[j] * weights[j];
//	pose.topRightCorner<3,1>() = pos;
//
//	// convert quat to rotation matrix
//	Vector3d rotV(3);
//	for (int j=0; j < 3; j++) rotV[j] = js_msg->axes[0].data[j+3];
//	Eigen::Matrix3d rotM = math::eulerToMatrix(rotV, spnavRotOrder);
//	pose.topLeftCorner<3,3>() = rotM;
//
//	// set lower right corner just to be safe
//	pose(3,3) = 1.0;
//	// Free the liberty message
//	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
//
//	// assume it's a spacenav:
//	pose = T_spacenav.inverse() * pose * T_spacenav;
//
//	// set the config if asked for
//	//if(config != NULL) *config << -pos(1), -pos(0), -pos(2), -rotV(2), -rotV(0), -rotV(1);
//	if(config != NULL) *config << -pos(1), -pos(0), -pos(2), -rotV(1), -rotV(0), -rotV(2);
//
//	return 0;
	
	Vector6d spnconfig = getSpacenavConfig(chan);
	pose = eulerToTransform(spnconfig, dartRotOrder);

	if (config != NULL)
		*config = spnconfig;
}


#endif /* JOYSTICK_CLIENT_H_ */
