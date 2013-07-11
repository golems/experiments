/**
 * @file 03-wheels.cpp
 * @author Can Erdogan
 * @date July 11, 2013
 * @brief This demonstration simple moves the wheels back and forth, and spin to show how to use 
 * of the wheels.
 */
#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <somatic/motor.h>

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Initialize the motor and daemons
void init() {

}

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}
