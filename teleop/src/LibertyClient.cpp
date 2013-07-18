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

void LibertyClient::initLiberty(somatic_d_t *daemon_cx, const char* chan_name) {
	somatic_d_channel_open(daemon_cx, &liberty_chan, chan_name, NULL);
}

bool LibertyClient::getLibertyPoses(Eigen::MatrixXd* poses[], size_t n_chan, int* chan_ids) {


}
