/*
 * joystick_client.h
 *
 *  Created on: Jul 14, 2013
 *      Author: jscholz
 */

#ifndef JOYSTICK_CLIENT_H_
#define JOYSTICK_CLIENT_H_


#include <ach.h>
#include "somatic.h"
#include <somatic.pb-c.h>
#include <Eigen/Dense>
#include <math/UtilsRotation.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

// Channel variables
uint8_t *achbuf_joystick;
size_t n_achbuf_joystick = 1024;
ach_channel_t joystick_chan; // global b/c why would you change it
size_t joystick_id = 1;

MatrixXd T_spacenav = (MatrixXd(4,4) <<  0,-1,0,0, -1,0,0,0, 0,0,-1,0, 0,0,0,1).finished();

void initJoystick() {
	ach_init(&joystick_chan, "joystick-data", achbuf_joystick, n_achbuf_joystick);
}

/*
 * Return the joystick pose in the world frame
 *TODO: rename/refactor for spacenav and joy versions
 */
bool getJoystickPose(MatrixXd &pose, Vector6d* config = NULL) {
	// get joystick data
	int r = 0;
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
			&protobuf_c_system_allocator,
			4096, &joystick_chan );

	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return -1;

	// scale dimensions before we start
	double weights[] = {1, 1, 1, 1, 1, 1};

	// extract translation
	Vector3d pos(3);
	for (int j=0; j < 3; j++)
		pos[j] = js_msg->axes[0].data[j] * weights[j];
	pose.topRightCorner<3,1>() = pos;

	// convert quat to rotation matrix
	Vector3d rotV(3);
	for (int j=0; j < 3; j++) rotV[j] = js_msg->axes[0].data[j+3];
	Eigen::Matrix3d rotM = math::eulerToMatrix(rotV, math::XYZ); //spnav
	pose.topLeftCorner<3,3>() = rotM;

	// set lower right corner just to be safe
	pose(3,3) = 1.0;
	// Free the liberty message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);

	// assume it's a spacenav:
	pose = T_spacenav.inverse() * pose * T_spacenav;

	// set the config if asked for
	if(config != NULL) *config << -pos(1), -pos(0), -pos(2), -rotV(2), -rotV(0), -rotV(1);
	
	return 0;
}


#endif /* JOYSTICK_CLIENT_H_ */
