/**
 * @file 06-waist.cpp
 * @author Can Erdogan
 * @date Jan 03, 2014
 * @brief This executable sends current commands to the waist modules.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include "somatic/motor.h"
#include <somatic.pb-c.h>
#include <ach.h>

#include <argp.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string>
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std;

// The somatic context and options
somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;

#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define CURRENT SOMATIC__MOTOR_PARAM__MOTOR_CURRENT

ach_channel_t stateChan;				///< The state channel, info, for the motors
ach_channel_t cmdChan;					///< The command channels (we send)
somatic_motor_t motors;					///< The motors we will set velocity/current values

Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		///< The waist command
Somatic__WaistMode mode;
double current;

/* ********************************************************************************************* */
double getWaistPos () {

	// Get the data
	int r;
	struct timespec abstime = aa_tm_future( aa_tm_sec2timespec( 1.0 / 30.0 ));
	Somatic__MotorState *waistState = SOMATIC_WAIT_LAST_UNPACK(r, somatic__motor_state, 
		&protobuf_c_system_allocator, 1024, &stateChan, &abstime);
	aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME, 
		"Ach wait failure %s on pcio data receive (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Clean up and return
	double pos = waistState->position->data[0];  
	somatic__motor_state__free_unpacked(waistState, &protobuf_c_system_allocator);
	return pos;
}

/* ********************************************************************************************* */
void run () {
	
	// Send a message; set the event code and the priority
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Send commands in either joystick or current mode
		somatic_waist_cmd_set(waistDaemonCmd, mode);
		if(mode == SOMATIC__WAIST_MODE__REAL_CURRENT_MODE) 
			somatic_vector_set_data(waistDaemonCmd->data, &current, 1);
		int r = SOMATIC_PACK_SEND(&cmdChan, somatic__waist_cmd, waistDaemonCmd);
		if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
			ach_result_to_string(static_cast<ach_status_t>(r)));

		cout << "pos: " << getWaistPos() << endl;
		usleep(1e5);
	
		aa_mem_region_release(&somaticContext.memreg);
	}

	// Send the stopping event
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {

	// Send message to the krang-waist daemon
	static Somatic__WaistCmd *waistDaemonCmd=somatic_waist_cmd_alloc();
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
	somatic_metadata_set_time_now(waistDaemonCmd->meta);
	somatic_metadata_set_until_duration(waistDaemonCmd->meta, .1);
	ach_status_t r = SOMATIC_PACK_SEND( &cmdChan, somatic__waist_cmd, waistDaemonCmd );
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", ach_result_to_string(r));

	// Close the values
	somatic_d_channel_close(&somaticContext, &stateChan);
	somatic_d_channel_close(&somaticContext, &cmdChan);

	// End the daemon
	somatic_d_destroy(&somaticContext);
}

/* ********************************************************************************************* */
void init(const int argc, char** argv) {
	
	// Initialize the somatic context
	somatic_d_init(&somaticContext, &somaticOptions);
	
	// Open channels for the waist daemon 
	somatic_d_channel_open(&somaticContext, &stateChan, "waist-state", NULL);
	somatic_d_channel_open(&somaticContext, &cmdChan, "waistd-cmd", NULL);

	// Interpret the input arguments for the mode
	assert(argc > 1 && "Need to give at least a mode");
	int mode_idx = atoi(argv[1]);
	if(mode_idx == 0) mode = SOMATIC__WAIST_MODE__MOVE_FWD;
	else if(mode_idx == 1) mode = SOMATIC__WAIST_MODE__MOVE_REV;
	else if(mode_idx == 2) mode = SOMATIC__WAIST_MODE__STOP;
	else if(mode_idx == 3) mode = SOMATIC__WAIST_MODE__CURRENT_MODE;
	else if(mode_idx == 4) mode = SOMATIC__WAIST_MODE__REAL_CURRENT_MODE;
	else assert(false && "Unknown mode");

	// If the mode is current, look for the next argument for the current value
	if(mode_idx == 4) {
		assert(argc > 2 && "Need to give a current value with the 'current' mode");
		current = atof(argv[2]);	
	}

	printf("Sleeping for a second!\n");
	usleep(1e6);
}

/* ********************************************************************************************* */
int main(int argc, char ** argv) {

	// Set the somatic context options
	somaticOptions.ident = "06-waist";
	somaticOptions.sched_rt = SOMATIC_D_SCHED_NONE; // logger not realtime
	somaticOptions.skip_mlock = 0; // logger not realtime, other daemons may be

	init( argc, argv );
	run();
	destroy();
	return 0;
}
