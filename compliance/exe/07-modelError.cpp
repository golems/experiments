/**
 * @file 07-modelError.cpp
 * @author Can Erdogan
 * @date June 15, 2013
 * @brief This executable aims at analyzing the modeling error observed in experiments 02 and 03,
 * and documented in debugging-amplitude folder under data. The main observation is that the
 * f/t sensor seems to return weird signals when there is no mass on it. We want to create a 
 * table of what these signals so we can subtract them in runtime, assuming that they stay a 
 * constant, which we have seen in experiment-1 of 03-estimation.
 */

#include "helpers.h"
#include <Eigen/StdVector>

using namespace std;
using namespace dynamics;
using namespace simulation;

#define DEG2RAD(x) (((x) / 180.0) * M_PI)
#define M_2PI (2 * M_PI)
#define NUM_READINGS 100
#define fix(x) ((fabs(x) < 1e-5) ? 0 : x)


/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;
Vector6d offset;							
World* mWorld = NULL;

vector <Vector3d, aligned_allocator<Vector3d> > goals;		///< Last 3 dof that the arm will visit
vector <Vector6d, aligned_allocator<Vector6d> > readings;	///< Mean readings at the goal locations

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0, currGoal = 0;
	Vector6d raw, external;
	Matrix3d Rsb;	//< The sensor frame in bracket frame (only rotation)
	double q [] = {0.0, -M_PI_2, 0.0, 0.0, 0.0, M_PI_2, 0.0};	
	size_t numReadings = 0;
	vector <Vector6d, aligned_allocator<Vector6d> > localReadings;
	localReadings.resize(NUM_READINGS);
	while(!somatic_sig_received) {
		
		c++;

		// Check if reached all the goals
		if(currGoal == goals.size()) {
			cout << "done!" << endl;
			break;
		}

		// Check if the arm has reached the desired position
		somatic_motor_update(&daemon_cx, &llwa);
		Vector3d error = Vector3d(llwa.pos[4], llwa.pos[5], llwa.pos[6]) - goals[currGoal];
		double maxError = error.cwiseAbs().maxCoeff();
		bool reached = (fabs(maxError) < 1e-2);

		// If reached the position, get enough readings to average over
		if(reached) {

			// Get a new reading
			bool gotReading = false;
			while(!gotReading) gotReading = getFT(daemon_cx, ft_chan, raw);
			localReadings[numReadings] = raw;	
			numReadings++;

			// Increment the goal index if you got enough readings
			if(numReadings >= NUM_READINGS) {

				// Average the readings
				Vector6d mean = Vector6d::Zero();
				for(size_t i = 0; i < NUM_READINGS; i++) mean += localReadings[i];
				mean /= NUM_READINGS;

				// Write the reading and the joint values to a file
				cout << goals[currGoal].transpose() << " " << mean.transpose() << endl;

				// Get the next goal
				currGoal++;
				numReadings = 0;
			}
		}

		// If did not reach, move towards the goal
		for(size_t i = 0; i < 3; i++) q[i+4] = (goals[currGoal])(i);
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_POSITION, q, 7, NULL);

		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Kills the motor and daemon structs 
void destroy() {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_d_destroy(&daemon_cx);
}

/* ******************************************************************************************** */
/// Generates the joint positions to reach discretized orientations of the end-effector
void generateJointValues () {

	std::vector <int> arm_ids;		///< The index vector to set config of arms
	int arm_ids_a [] = {10, 12, 14, 16, 18, 20, 22};
	for(size_t i = 0; i < 7; i++) arm_ids.push_back(arm_ids_a[i]);

	// The separation between point samples where dth3 is the smallest angle
	// double dth1 = -DEG2RAD(12.0), dth2 = DEG2RAD(12.0), dth3 = -DEG2RAD(30.0);
	double dth1 = -DEG2RAD(120.0), dth2 = DEG2RAD(45.0), dth3 = -DEG2RAD(180.0);

	// We first fix the vertical joint - this basically determines the radius of the circle we
	// draw in the yz plane, starting from big to small
	bool turn1 = 0, turn3 = 0;
	double th1Low = M_2PI, th1High = 0.0;
	double th3Low = M_2PI, th3High = 0.0;
	for(double th2 = M_PI_2; th2 >= 0; th2 = fix(th2 - dth2)) {

		// Decide on the direction th1 is going to rotate at to avoid joint limits
		dth1 *= -1;
		th1Low = M_2PI - th1Low;
		th1High = M_2PI - th1High;
		turn1 = !turn1;
		
		// Then, we move the biggest joint, th1, to create a circle for the ee position
		for(double th1 = th1Low; turn1 ? (th1 < th1High) : (th1 > th1High); th1 = fix(th1 + dth1)) {

			// Again decide on direction to avoid joint limits
			dth3 *= -1;
			th3Low = M_2PI - th3Low;
			th3High = M_2PI - th3High;
			turn3 = !turn3;
	
			// If th1 and th3 are aligned, there is no need to sample th3
			if(th2 == 0.0) {
				goals.push_back(Vector3d(0.0, th2, th1));
				// cout << goals.back().transpose() << endl;
				continue;
			}

			// Lastly, for any fixed location, we rotate the smallest joint
			for(double th3 = th3Low; turn3 ? (th3 < th3High) : (th3 > th3High); th3 = fix(th3 + dth3)) {
				goals.push_back(Vector3d(th1, th2, th3));
				// cout << goals.back().transpose() << endl;
			}
		}
	}

	cout << "Generated " << goals.size() << " goals!\n";
}

/* ******************************************************************************************** */
void init () {

	// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
	// NOTE: We do not want to filter this data yet and we want it raw
	system("killall -s 9 netcanftd");
	usleep(20000);
	system("netcanftd -v -d -I lft -b 2 -B 1000 -c llwa_ft -r");

	// Initialize this daemon (program!)
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "01-gripperWeight";
	somatic_d_init( &daemon_cx, &dopt );

	// Initialize the left arm
	initArm(daemon_cx, llwa, "llwa");
	somatic_motor_update(&daemon_cx, &llwa);

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
	// Open the state and ft channels 
	somatic_d_channel_open(&daemon_cx, &ft_chan, "llwa_ft", NULL);

	// Generate joint values to visit
	generateJointValues();
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	init();
	run();
	destroy();
	return 0;
}

