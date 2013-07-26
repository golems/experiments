/*
 * liberty_client.h
 *
 *  Created on: Jul 14, 2013
 *      Author: jscholz
 */

#ifndef LIBERTY_CLIENT_H_
#define LIBERTY_CLIENT_H_

#include <somatic.h>
#include <somatic.pb-c.h>
#include <Eigen/Dense>

#include "channel.h"

// Channel variables
uint8_t *achbuf_liberty;
size_t n_achbuf_liberty = 1024;
ach_channel_t liberty_chan; // global b/c why would you change it

void initLiberty() {
	ach_init(&liberty_chan, "liberty", achbuf_liberty, n_achbuf_liberty);
}

/*
 * Queries liberty ach channel for current liberty channels 1 and 2,
 * and converts them to MatrixXd transforms.
 */
bool getLibertyPoses(MatrixXd *poses[], size_t n_chan, int *chan_ids = NULL) {
	// get liberty data
	int r = 0;
	Somatic__Liberty *ls_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__liberty,
			&protobuf_c_system_allocator,
			4096, &liberty_chan );

	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (ls_msg == NULL)) return -1;

	// stack sensors into a single array for indexing
	Somatic__Vector* sensors[] = {ls_msg->sensor1, ls_msg->sensor2, ls_msg->sensor3, ls_msg->sensor4,
			ls_msg->sensor5, ls_msg->sensor6, ls_msg->sensor7, ls_msg->sensor8};

	// pack liberty data into arrowConfs
	for (int i=0; i < n_chan; ++i) {
		Somatic__Vector* sensor = sensors[i];
		VectorXd pos(3);
		for (int j=0; j < 3; j++) pos[j] = sensor->data[j];
		poses[i]->topRightCorner<3,1>() = pos;

		// convert quat to rotation matrix
		Eigen::Quaternion<double> rotQ(&sensor->data[3]);
		Matrix3d rotM(rotQ);
		poses[i]->topLeftCorner<3,3>() = rotM;

		// set lower right corner just to be safe
		(*poses[i])(3,3) = 1.0;
	}

	// Free the liberty message
	somatic__liberty__free_unpacked(ls_msg, &protobuf_c_system_allocator);

	return 0;
}

#endif /* LIBERTY_CLIENT_H_ */
