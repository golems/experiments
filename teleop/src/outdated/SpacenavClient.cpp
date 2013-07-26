/*
 * SpacenavTeleop.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#include "SpacenavClient.h"


#include <math/UtilsRotation.h>

#include "util.h"

SpacenavClient::SpacenavClient() {
	// TODO Auto-generated constructor stub

}

SpacenavClient::~SpacenavClient() {
	// TODO Auto-generated destructor stub
}

void SpacenavClient::initialize(somatic_d_t *daemon_cx, const char* channel_name) {

	this->daemon_cx = daemon_cx;
	somatic_d_channel_open(daemon_cx, &spacenav_chan, channel_name, NULL);
}

void SpacenavClient::setInitialTransform() {
	//T_spn_init =
}

Eigen::VectorXd SpacenavClient::getConfig(double pos_scale, double rot_scale) {

	Eigen::VectorXd config(6); config.setZero();

	// get joystick data
	int r = 0;
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
			&protobuf_c_system_allocator,
			4096, &spacenav_chan );

	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return config;

	// extract translation
	Eigen::Vector3d pos(3);
	for (int j=0; j < 3; j++)
		pos[j] = js_msg->axes[0].data[j] * pos_scale;

	// convert quat to rotation matrix
	Eigen::Vector3d rotV(3);
	for (int j=0; j < 3; j++)
		rotV[j] = js_msg->axes[0].data[j+3] * rot_scale;

	// pack into config (spacenav has weird frame when read as a joystick)
	config << -pos(1), -pos(0), -pos(2), -rotV(1), -rotV(0), -rotV(2);

	// Free the liberty message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);

	return config;
}

bool SpacenavClient::getPose(Eigen::MatrixXd& pose, Eigen::VectorXd* config) {

	Eigen::VectorXd spnconfig = getConfig();
	pose = eulerToTransform(spnconfig, math::XYZ);

	if (config != NULL)
		*config = spnconfig;

	return 0;
}

Eigen::VectorXi SpacenavClient::getButtons() {

	Eigen::VectorXi buttons(2); buttons.setZero(2);

	// get spacenav data
	int r = 0;
	Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
			&protobuf_c_system_allocator,
			4096, &spacenav_chan );
//cout << ach_result_to_string(static_cast<ach_status_t>(r)) << endl;
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r || ACH_STALE_FRAMES == r) || (js_msg == NULL)) return buttons;

	// pack output vector
//	cout << "button0 raw: " << js_msg->buttons->data[0] << endl;
//	cout << "button1 raw: " << js_msg->buttons->data[1] << endl;
	buttons << js_msg->buttons->data[0], js_msg->buttons->data[1];

	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);

	return buttons;
}
