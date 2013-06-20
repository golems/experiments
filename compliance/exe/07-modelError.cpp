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

using namespace std;
using namespace dynamics;
using namespace simulation;

#define DEG2RAD(x) (((x) / 180.0) * M_PI)
#define M_2PI (2 * M_PI)

/* ********************************************************************************************* */
somatic_d_t daemon_cx;
ach_channel_t js_chan;				
ach_channel_t ft_chan;
somatic_motor_t llwa;
Vector6d offset;							
World* mWorld = NULL;

vector <Vector3d> goals;				///< The goal locations for the last 3 dof that the arm will visit
vector <Vector6d> readings;			///< The average readings at the goal locations

/* ******************************************************************************************** */
/// The continuous loop
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	size_t c = 0;
	Vector6d raw, external;
	Matrix3d Rsb;	//< The sensor frame in bracket frame (only rotation)
	while(!somatic_sig_received) {
		
		c++;

		// Move the arm to any position with the joystick
		setJoystickInput(daemon_cx, js_chan, llwa, llwa);
		somatic_motor_update(&daemon_cx, &llwa);
	
		// Get the f/t sensor data and compute the ideal value
		size_t k = 1e6;
		bool result = (c % k == 0) && getFT(daemon_cx, ft_chan, raw);
		if(!result) continue;

		// Compute the ideal value
		Vector6d ideal = raw + offset;

		// Compute the external forces from ideal readings
		computeExternal(llwa, ideal, *(mWorld->getSkeleton(0)), external);
		pv(external);

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
	double dth1 = -DEG2RAD(6.0), dth2 = DEG2RAD(6.0), dth3 = -DEG2RAD(18.0);

	// We first fix the vertical joint - this basically determines the radius of the circle we
	// draw in the yz plane, starting from big to small
	bool turn1 = 0, turn3 = 0;
	double th1Low = M_2PI, th1High = 0.0;
	double th3Low = M_2PI, th3High = 0.0;
	for(double th2 = M_PI_2; th2 >= 0; th2 -= dth2) {

		// Decide on the direction th1 is going to rotate at to avoid joint limits
		dth1 *= -1;
		th1Low = M_2PI - th1Low;
		th1High = M_2PI - th1High;
		turn1 = !turn1;
		
		// Then, we move the biggest joint, th1, to create a circle for the ee position
		for(double th1 = th1Low; turn1 ? (th1 <= th1High) : (th1 >= th1High); th1 += dth1) {

			// Again decide on direction to avoid joint limits
			dth3 *= -1;
			th3Low = M_2PI - th3Low;
			th3High = M_2PI - th3High;
			turn3 = !turn3;
	
			// Lastly, for any fixed location, we rotate the smallest joint
			for(double th3 = th3Low; turn3 ? (th3 <= th3High) : (th3 >= th3High); th3 += dth3) {
				goals.push_back(Vector3d(th1, th2, th3));
				Vector7d vals;
				vals << 0.0, -M_PI_2, 0.0, 0.0, th1, th2, th3;
//				mWorld->getSkeleton(0)->setConfig(arm_ids, vals);
//				cout << mWorld->getSkeleton(0)->getNode("lGripper")->getWorldTransform().topRightCorner<3,1>().transpose() << endl;
				cout << goals.back().transpose() << endl;
			}
		}
	}
}

/* ******************************************************************************************** */
/// The main thread
int main() {
	DartLoader dl;
	mWorld = dl.parseWorld("../scenes/01-World-Robot.urdf");
	assert((mWorld != NULL) && "Could not find the world");
	generateJointValues();
	return 0;
	init(daemon_cx, js_chan, ft_chan, llwa, offset);
	run();
	destroy();
	return 0;
}

