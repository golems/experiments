/*
 * LibertyClient.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#include "LibertyClient.h"

LibertyClient::LibertyClient() {
	// TODO Auto-generated constructor stub

}

LibertyClient::~LibertyClient() {
	// TODO Auto-generated destructor stub
}

void LibertyClient::initLiberty() {
	int r = ach_open(&js_chan, channel_name, NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
}

bool LibertyClient::getLibertyPoses(Eigen::MatrixXd* poses[], size_t n_chan,
		int* chan_ids) {
}
