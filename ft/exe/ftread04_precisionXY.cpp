/**
 * @file ftread04_precisionXY.cpp
 * @author Munzir Zafar, Can Erdogan
 * @date Dec 20, 2012
 * @brief This program evaluates the accuracy of force/torque measurements read from the given 
 * channel. A known mass is hung at the end-effector perpendicular to z-axis at a known angle 
 * w.r.t. x-axis. This program then predicts the Fx, Fy components and compares them with the
 * average values of Fx and Fy being read from the channel. It also evaluates the standard 
 * deviation, minimum and maximum of the signed error i.e. we take into account the direction 
 * of deviation as well.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

#define GRAVITY 9.80665 // m/s2

// The somatic context and options
somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;

// The ach channel and its name
ach_channel_t achChannel;
const char *channelName = NULL;

// Default values for the input arguments
double mass = 0.0;			// Mass of the experimented weight
double theta = 0.0;			// The angle wrt to x-axis
size_t sampleSize = 100;	// Sample size for stdev calculation
size_t nIter = 5;			// Number of iterations for repeated calculation

// Predictions for the measurements (calculated in init())
float predictedFx;	
float predictedFy;

// Structure to hold the errors
float** errors;

/* ********************************************************************************************* */
// argp options
static struct argp_option options[] = { 
	{"chan", 'c', "ach channel", 0, "ach channel to receive the force/torque data from"}, 
	{"mass", 'm', "loaded mass", 0, "mass loaded on the end effector in kg"},
	{"angle", 'a', "direction of force", 0, 
		"angle between the gravitational force and positive x-axis in degrees"},
	{"samples", 'n', "number of samples", 0, "every n samples will be evaluated to find the mean,"
		 " standard deviation, minimum and maximum values" },
	{"iter", 'i', "iterations", 0, "number of evaluations to be performed"},
	{0} 
};

/* ********************************************************************************************* */
// argp constants
static int parse_opt( int key, char *arg, struct argp_state *state);
static char args_doc[] = "This program evaluates the accuracy of F/T measurements";
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
	case 'm':
		mass = atof(arg);
		printf("mass = %f\n", mass);
		break;
	case 'a':
		theta = atof(arg);
		printf("theta = %f\n", theta);
		break;
	case 'n':
		sampleSize = atoi(arg);
		printf("sampleSize = %d\n", sampleSize);
		break;
	case 'i':
		nIter = atoi(arg);
		printf("iterations = %d\n", nIter);
		break;
	}

	// Flush the input arguments
	fflush(stdout);
	return 0;
}

/* ********************************************************************************************* */
void init() {
	// Initialize the somatic context 
	// NOTE: somatic_d_init sends a SOMATIC__EVENT__CODES__PROC_STARTING message on the ach channel.
	// NOTE: somatic_d_channel opens the ach channel to read from in update(). 
	somatic_d_init(&somaticContext, &somaticOptions);
	somatic_d_channel_open(&somaticContext, &achChannel, channelName, NULL);
	
	// Predict the values for Fx and Fy
	float weight = GRAVITY * mass;
	predictedFx = weight * cos(theta*M_PI/180.0);	
	predictedFy = weight * sin(theta*M_PI/180.0);
	printf("Predictions: Fx = %3.2f, Fy = %3.2f \n", predictedFx, predictedFy);

	// Initialize a structure for storing sample data
	errors = new float*[2];
	errors[0] = new float[sampleSize];
	errors[1] = new float[sampleSize];
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

	// Read the force-torque message
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(somaticContext.pballoc), numBytes, buffer);
	static size_t count = 0;

	// Update the structure
	errors[0][count] = ftMessage->force->data[0] - predictedFx;
	errors[1][count] = ftMessage->force->data[1] - predictedFy;
	
	// Evaluate the samples 
	static size_t iteration = 1;
	if(count++ == sampleSize) {
		count = 0;
	
		// Calculate average, min, max
		float meanErrorFx=0.0, meanErrorFy=0.0; 
		float minErrorFx = 1000.0, maxErrorFx = -1000.0;
		float minErrorFy = 1000.0, maxErrorFy = -1000.0;	
		for(int i = 0; i<sampleSize; i++){
			meanErrorFx += errors[0][i];
			minErrorFx = fmin(minErrorFx, errors[0][i]);
			maxErrorFx = fmax(maxErrorFx, errors[0][i]);
		
			meanErrorFy += errors[1][i];
			minErrorFy = fmin(minErrorFy, errors[1][i]);
			maxErrorFy = fmax(maxErrorFy, errors[1][i]);
		}
		meanErrorFx /= sampleSize;
		meanErrorFy /= sampleSize;
		
		// Calculate standard deviation
		float stdDevErrorFx=0.0, stdDevErrorFy=0.0;
		for(int i=0; i<sampleSize; i++ ){
			stdDevErrorFx += pow((errors[0][i]-meanErrorFx),2.0);
			stdDevErrorFy += pow((errors[1][i]-meanErrorFy),2.0);
		}
		stdDevErrorFx = sqrt(stdDevErrorFx/sampleSize);
		stdDevErrorFy = sqrt(stdDevErrorFy/sampleSize);
		
		// Print out the evaluation
		printf("Iteration: %d\n" , iteration);
		printf("\tFx errors: mean = %3.2f, stdev = %3.2f, min = %3.2f, max = %3.2f\n",
			meanErrorFx, stdDevErrorFx, minErrorFx, maxErrorFx);
		printf("\tFy errors: mean = %3.2f, stdev = %3.2f, min = %3.2f, max = %3.2f\n\n",
			meanErrorFy, stdDevErrorFy, minErrorFy, maxErrorFy);
		fflush(stdout);

		// Quit if reached desired number of iterations
		if(iteration++ == nIter) exit(EXIT_SUCCESS);	
	}
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
