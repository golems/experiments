/*
 * LibertyClient.h
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#ifndef LIBERTYCLIENT_H_
#define LIBERTYCLIENT_H_

#include <amino.h>
#include <ach.h>
#include <somatic.h>
#include <somatic.pb-c.h>
#include <somatic/daemon.h>
#include <Eigen/Dense>

class LibertyClient {
public:
	LibertyClient();
	virtual ~LibertyClient();

	// initialization
	void initLiberty(somatic_d_t *daemon_cx, const char* channel_name = "liberty");

	// update methods
	bool getLibertyPoses(Eigen::MatrixXd *poses[], size_t n_chan, int *chan_ids = NULL);


protected:
	// Channel members
	somatic_d_t daemon_cx;
	uint8_t *achbuf_liberty;
	static const size_t n_achbuf_liberty = 1024;
	ach_channel_t liberty_chan; // global b/c why would you change it

	std::vector<Eigen::Matrix4d> initPoses;
	std::vector<Eigen::Matrix4d> curPoses;

};

#endif /* LIBERTYCLIENT_H_ */
