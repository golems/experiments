/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** \file LibertyClient.cpp
 *
 *  \author Jonathan Scholz
 *  \date 7/17/2013
 */

#include "LibertyClient.h"
#include <math/UtilsRotation.h>
#include "util.h" //TODO: rename, move, or do something good

LibertyClient::LibertyClient() {}

LibertyClient::~LibertyClient() {}

void LibertyClient::initLiberty(somatic_d_t *daemon_cx, const char* chan_name,
		size_t liberty_n_channels, int *liberty_chan_ids) {

	this->daemon_cx = daemon_cx;

	initPoses.resize(liberty_n_channels);
	rawPoses.resize(liberty_n_channels);
	relPoses.resize(liberty_n_channels);

	this->liberty_chan_ids = Eigen::VectorXi(liberty_n_channels);
	for (size_t i=0; i<liberty_n_channels; i++)
		this->liberty_chan_ids[i] = liberty_chan_ids[i];

	somatic_d_channel_open(daemon_cx, &liberty_ach_chan, chan_name, NULL);
}

void LibertyClient::setInitialPoses() {
	updateRawPoses();

	for (int i=0; i < initPoses.size(); i++) {
		initPoses[i] = rawPoses[i]; // set the whole thing

		// just set the position offset
//		initPoses[i].setIdentity();
//		initPoses[i].topRightCorner<3,1>() = rawPoses[i].topRightCorner<3,1>();
	}

}

bool LibertyClient::updateRawPoses() {
	// get liberty data
	int r = 0;
	Somatic__Liberty *ls_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__liberty,
			&protobuf_c_system_allocator,
			n_achbuf_liberty, &liberty_ach_chan );

	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (ls_msg == NULL)) return -1;

	// stack sensors into a single array for indexing
	Somatic__Vector* sensors[] = {ls_msg->sensor1, ls_msg->sensor2, ls_msg->sensor3, ls_msg->sensor4,
			ls_msg->sensor5, ls_msg->sensor6, ls_msg->sensor7, ls_msg->sensor8};

	// pack liberty data into arrowConfs
	for (int i=0; i < liberty_chan_ids.size(); i++) {
		Somatic__Vector* sensor = sensors[i];

		Eigen::VectorXd config(6);
		config << -sensor->data[0], sensor->data[1], sensor->data[2],
				-sensor->data[5], -sensor->data[4], sensor->data[3];

		rawPoses[liberty_chan_ids[i]] = eulerToTransform(config, math::XYZ);
	}

	// Free the liberty message
	somatic__liberty__free_unpacked(ls_msg, &protobuf_c_system_allocator);

	return 0;
}

void LibertyClient::updateRelPoses() {
	for (int i=0; i < relPoses.size(); i++)
		relPoses[i] = initPoses[i].inverse() * rawPoses[i];
}
