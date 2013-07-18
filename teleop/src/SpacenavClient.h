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
class SpacenavClient {
public:
	SpacenavClient();
	virtual ~SpacenavClient();

	// initialization
	void initialize(somatic_d_t *daemon_cx, const char* channel_name = "spacenav-data");
	void setInitialTransform();

	// update methods
	Eigen::VectorXd getConfig(double pos_scale = 1.0, double rot_scale = 1.0);
	Eigen::VectorXi getButtons();
	bool getPose(Eigen::MatrixXd &pose, Eigen::VectorXd* config = NULL);

protected:
	// Channel variables
	somatic_d_t *daemon_cx;  ///< somatic daemon pointer
	uint8_t *achbuf_joystick;
	ach_channel_t spacenav_chan;

	// Transforms
	Eigen::Matrix4d T_spn_init; //< initial spacenav transform
	Eigen::Matrix4d T_spn_cur; //< current spacenav transform
};

#endif /* SPACENAVTELEOP_H_ */
