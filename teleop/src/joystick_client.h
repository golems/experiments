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

// Channel variables
uint8_t *achbuf_joystick;
size_t n_achbuf_joystick = 1024;
ach_channel_t joystick_chan; // global b/c why would you change it
size_t joystick_id = 1;

MatrixXd T_spacenav = (MatrixXd(4,4) <<  0,-1,0,0,1,0,0,0,0,0,-1,0,0,0,0,1).finished();

void initJoystick() {
	ach_init(&joystick_chan, "joystick-data", achbuf_joystick, n_achbuf_joystick);
}

/*
 * Return the joystick pose in the world frame
 *
 */
bool getJoystickPose(MatrixXd &pose) {
	// get joystick data
	int r = 0;
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
			&protobuf_c_system_allocator,
			4096, &joystick_chan );

	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return -1;

	// extract translation
	Vector3d pos(3);
	for (int j=0; j < 3; j++) pos[j] = js_msg->axes[0].data[j];
	pose.topRightCorner<3,1>() = pos;

	// convert quat to rotation matrix
	Vector3d rotV(3);
	for (int j=0; j < 3; j++) rotV[j] = js_msg->axes[0].data[j+3];
	Eigen::Matrix3d rotM = math::eulerToMatrix(rotV, math::XYZ);
	pose.topLeftCorner<3,3>() = rotM;

	// set lower right corner just to be safe
	pose(3,3) = 1.0;
	// Free the liberty message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);

	// assume it's a spacenav:
	std::cout << std::endl << "raw pose: " << std::endl << pose << std::endl; return 0;
	//pose = T_spacenav * pose;
	std::cout << std::endl << "spacenav pose: " << std::endl << T_spacenav * pose << std::endl; return 0;
}


#endif /* JOYSTICK_CLIENT_H_ */
