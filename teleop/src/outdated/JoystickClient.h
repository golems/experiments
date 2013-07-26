/*
 * JoystickClient.h
 *
 *  Created on: Jul 17, 2013
 *      Author: jscholz
 */

#ifndef JOYSTICKCLIENT_H_
#define JOYSTICKCLIENT_H_

#include <amino.h>
#include <ach.h>
#include <somatic.h>
#include <somatic.pb-c.h>
#include <somatic/daemon.h>
#include <Eigen/Dense>


class JoystickClient {
public:
	JoystickClient();
	virtual ~JoystickClient();

	// initialization
	void initialize(somatic_d_t *daemon_cx, const char* channel_name = "joystick-data");

	// update methods
	void getLatest();
	void getButtonsAndAxes(Eigen::VectorXi &buttons, Eigen::VectorXd &axes);
	Eigen::VectorXi getButtons(bool update = true);
	Eigen::VectorXd getAxes(bool update = true);

protected:
	// channel members
	somatic_d_t daemon_cx;
	ach_channel_t js_chan;

	// data members
	Eigen::VectorXd axes;
	Eigen::VectorXi buttons;
};

#endif /* JOYSTICKCLIENT_H_ */
