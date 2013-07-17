/*
 * SpacenavTeleop.h
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#ifndef SPACENAVTELEOP_H_
#define SPACENAVTELEOP_H_

#include <amino.h>
#include <ach.h>
#include <somatic.h>
#include <somatic.pb-c.h>
#include <somatic/daemon.h>

#include <Eigen/Dense>

/*
 * Implements a basic workspace control client for the spacenav sensor
 */
class SpacenavTeleop {
public:
	SpacenavTeleop();
	virtual ~SpacenavTeleop();

	// initialization
	void initialize(somatic_d_t *daemon_cx, const char* channel_name = "spacenav-data");

	// update methods
	Eigen::VectorXd getSpacenavConfig();
	Eigen::VectorXi getSpacenavButtons();
	bool getSpacenavPose(Eigen::MatrixXd &pose, Eigen::VectorXd* config = NULL);

protected:
	// Channel variables
	somatic_d_t *daemon_cx;  ///< somatic daemon pointer
	uint8_t *achbuf_joystick;
	size_t n_achbuf_joystick = 1024;
	ach_channel_t spacenav_chan;

	// Transforms
	Eigen::Matrix4d T_spn_init;; //< initial spacenav transform
	Eigen::Matrix4d T_spn_cur; //< current spacenav transform
};

#endif /* SPACENAVTELEOP_H_ */
