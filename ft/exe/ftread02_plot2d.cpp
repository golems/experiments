/**
 * @file ftread02_plot2d.cpp
 * @author Can Erdogan
 * @date Dec 10, 2012
 * @brief This program plots the force and torque measurements in a 2D time-varying graph.
 * @todo Add units to the plots.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>
#include <algorithm>

using namespace std;

// The somatic context and options
somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;

// The ach channel and its name
ach_channel_t achChannel;
const char *channelName = NULL;

// Sampling option
size_t samplingRate = 0;

// Plotting variables
FILE* gnuplot;
int windowSize = 100;		// "int" because we need to use it in subtraction
float** data;

/* ********************************************************************************************* */
// argp options
static struct argp_option options[] = { 
	{"chan", 'c', "ach channel", 0, "ach channel to receive the force/torque data from"}, 
	{"sample", 's', "sampling rate", 0, "every nth value will be printed to the console"}, 
	{"window", 'w', "window size", 0, "the time range of the 2d plots"}, 
	{0} 
};

/* ********************************************************************************************* */
// argp constants
static int parse_opt( int key, char *arg, struct argp_state *state);
static char args_doc[] = "This program prints the force/torque measurements read from a channel";
static struct argp argp = {options, parse_opt, args_doc, NULL, NULL, NULL, NULL };

/* ********************************************************************************************* */
/// The parsing function that argp uses along with the given options
static int parse_opt( int key, char *arg, struct argp_state *state) {
	(void) state; // ignore unused parameter
	switch(key) {
	case 'c':
		if(strlen(arg) > ACH_CHAN_NAME_MAX) {
			fprintf(stderr, "ERROR: channel is too long\n");
			exit(1);
		} else {
			channelName = strdup(arg);
		}
		break;
	case 's':
		samplingRate = atoi(arg);
		break;
	case 'w':
		windowSize = atoi(arg);
		break;
	}
	return 0;
}

/* ********************************************************************************************* */
void init() {

	// Initialize the interface to the gnuplot setting some options
	gnuplot = popen("gnuplot -persist", "w");
//	gnuplot = fopen("bla", "w");
	fprintf(gnuplot, "set xlabel 'Time ()'\n");
	fprintf(gnuplot, "set xrange [0:100]\n");

	// Initialize the plotted data
	data = new float* [6];
	for(size_t i =0; i < 6; i++) {
		data[i] = new float [windowSize];
		for(size_t j = 0; j < windowSize; j++)
			data[i][j] = 0.0f;
	}

	// Initialize the somatic context 
	// NOTE: somatic_d_init sends a SOMATIC__EVENT__CODES__PROC_STARTING message on the ach channel.
	// NOTE: somatic_d_channel opens the ach channel to read from in update(). 
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
	uint8_t* buffer = (uint8_t*) somatic_d_get(&somaticContext, &achChannel, &numBytes, &abstimeout, ACH_O_WAIT, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return;

	// =======================================================
	// B. Read message

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(somaticContext.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return;
	if(msg->meta->type != SOMATIC__MSG_TYPE__FORCE_MOMENT) return;

	// Consider skipping the iteration if sampling
	static int counter = 0;
	counter++;
	if((samplingRate != 0) && ((counter % samplingRate) != 0)) return;

	// Read the force-torque message
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(somaticContext.pballoc), numBytes, buffer);

	// =======================================================
	// C. Plot the force and torque values

	// Set the data
	static int k = 0;
	k++;
	for(size_t p = 0; p < 3; p++) 
		data[p][k % windowSize] = ftMessage->force->data[p];
	for(size_t p = 0; p < 3; p++) 
		data[p+3][k % windowSize] = ftMessage->moment->data[p];

	// Update the range
	fprintf(gnuplot, "set xrange [%d:%d]\n", max(0, k-windowSize+1), k);

	// Plot the force values
	fprintf(gnuplot, "set term x11 0\n");
	fprintf(gnuplot, "set title 'Force Readings'\n");
	fprintf(gnuplot, "set yrange [-10:10]\n");
	fprintf(gnuplot, "plot '-' with points title 'fx'");	
	fprintf(gnuplot, ", '-' with points title 'fy'");	
	fprintf(gnuplot, ", '-' with points title 'fz'\n");	
	for(size_t p = 0; p < 3; p++) {
		for(int i = max(0, k - windowSize + 1); i <= k; i++)
			fprintf(gnuplot, "%d %f\n", i, data[p][i % windowSize]);
		fprintf(gnuplot, "e\n");
	}

	// Plot the torque values
	fprintf(gnuplot, "set term x11 1\n");
	fprintf(gnuplot, "set title 'Torque Readings'\n");
	fprintf(gnuplot, "set yrange [-2:2]\n");
	fprintf(gnuplot, "plot '-' with points title 'tx'");	
	fprintf(gnuplot, ", '-' with points title 'ty'");	
	fprintf(gnuplot, ", '-' with points title 'tz'\n");	
	for(size_t p = 3; p < 6; p++) {
		for(int i = max(0, k - windowSize + 1); i <= k; i++)
			fprintf(gnuplot, "%d %f\n", i, data[p][i % windowSize]);
		fprintf(gnuplot, "e\n");
	}
	fflush(gnuplot);

	
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

	// Close the channel and end the daemon
	somatic_d_channel_close(&somaticContext, &achChannel);
	somatic_d_destroy(&somaticContext);
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

	// Parse the arguments and make sure a channel name is given
	argp_parse (&argp, argc, argv, 0, NULL, NULL);
	if(channelName == NULL) {
		printf("Usage: ./ftread01_print -c <ach channel name>\n");
		exit(EXIT_FAILURE);
	}

	// Set the somatic context options
	somaticOptions.ident = "ftread01_print";
	somaticOptions.sched_rt = SOMATIC_D_SCHED_NONE; // logger not realtime
	somaticOptions.skip_mlock = 1; 		// not sure why?

	// Initialize the code and run until a somatic_sig is received (?)
	init();
	run();
	destroy();

	exit(EXIT_SUCCESS);
}
