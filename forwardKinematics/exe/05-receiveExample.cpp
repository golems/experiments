/**
 * @file server.cpp
 * @author Can Erdogan
 * @date Dec 09, 2012
 * @brief This file shows an example usage of the somatic library. The server
 * sends transform messages over the ach channel.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

/// argp program version
const char *argp_program_version = "server 0.0";
#define ARGP_DESC "writes somatic events to syslog"

// The somatic context and options
somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;

// The ach channel and its name
ach_channel_t achChannel;
const char *channelName;

// Argument processing
static int parse_opt( int key, char *arg, struct argp_state *state);
extern struct argp_option argp_options[];
extern struct argp argp;

/* ********************************************************************************************* */
void init() {
	somatic_opt_verbosity = 9;
	somatic_d_init(&somaticContext, &somaticOptions);
	somatic_d_channel_open(&somaticContext, &achChannel, channelName, NULL);
}

/* ********************************************************************************************* */
void update() {

	// =======================================================
	// A. Get message
	// NOTE: This is usually done with SOMATIC_D_GET which is a macro.

	// Set the time to read (?)
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&somaticContext, &achChannel, &numBytes, &abstimeout, ACH_O_LAST, &result);

	if(numBytes == 0) return;

	// Return if there is nothing to read
	// =======================================================
	// B. Read message

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(somaticContext.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return;
	if(msg->meta->type != SOMATIC__MSG_TYPE__TRANSFORM) return; 

	// Read the force-torque message
	Somatic__Transform* trMessage = somatic__transform__unpack(&(somaticContext.pballoc), numBytes, buffer);
	printf("[server] transform:\t");
	for(size_t i = 0; i < 3; i++)
		printf("%6.2f  ", trMessage->translation->data[i]); 
	for(size_t i = 0; i < 4; i++)
		printf("%6.2f  ", trMessage->rotation->data[i]); 
	printf("\n"); fflush(stdout);
}

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {
		update();
		aa_mem_region_release(&somaticContext.memreg);	// free buffers allocated during this cycle
	}

	// Send the stoppig event
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {
	somatic_d_channel_close(&somaticContext, &achChannel);
	somatic_d_destroy(&somaticContext);
}

/* ********************************************************************************************* */
int main() {

	// Set the somatic context options
	somaticOptions.ident = "server";
	somaticOptions.sched_rt = SOMATIC_D_SCHED_NONE; // logger not realtime
	somaticOptions.skip_mlock = 0; // logger not realtime, other daemons may be

	// Set the channel name
	channelName = "chan_transform";

	init();
	run();
	destroy();

	return 0;
}
