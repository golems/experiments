/*
 * JoystickClient.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#include "JoystickClient.h"

JoystickClient::JoystickClient() {
	// TODO Auto-generated constructor stub

}

JoystickClient::~JoystickClient() {
	// TODO Auto-generated destructor stub
}

void JoystickClient::initialize(somatic_d_t *daemon_cx, const char* channel_name) {
	int r = ach_open(&js_chan, channel_name, NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	axes = Eigen::VectorXd(6);
	buttons = Eigen::VectorXi(10);
}

void JoystickClient::getLatest() {
	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg =
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

	// Get the values
	for(size_t i = 0; i < 10; i++)
		buttons[i] = js_msg->buttons->data[i] ? 1 : 0;

	for(size_t i = 0; i < 6; i++)
		axes[i] = js_msg->axes->data[i];

	// Free the joystick message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
}

void JoystickClient::getButtonsAndAxes(Eigen::VectorXi &buttons, Eigen::VectorXd &axes) {
	getLatest();
	buttons = this->buttons;
	axes = this->axes;
}

Eigen::VectorXi JoystickClient::getButtons(bool update) {
	if (update)
		getLatest();
	return buttons;
}

Eigen::VectorXd JoystickClient::getAxes(bool update) {
	if (update)
		getLatest();
	return axes;
}
