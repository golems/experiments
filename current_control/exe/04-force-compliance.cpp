/** 
    May the force be with you... sometimes.
**/

#include "helpers.h"
#include "initModules.h"
#include "motion.h"
#include "current_control.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>

// DART crap
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

// ##############################
// Definitions

// ##############################
// Global variables

somatic_d_t daemon_cx;
ach_channel_t state_chan;
ach_channel_t cmd_chan;
ach_channel_t ft_chan;
ach_channel_t waist_chan; 
ach_channel_t imu_chan;

somatic_motor_t rlwa;
somatic_motor_t llwa;

Vector6d loffset, roffset;

bool use_pos[] = {true, true, true, true, true, true, true};
bool use_vel[] = {true, true, true, true, true, true, true};

double init_K_p_p[] = {15.0,  15.0, 15.0, 12.0, 15.0,  7.0,  7.0};
double init_K_p_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
double init_K_v_p[] = {1.0,   1.0,  1.0,  1.0,  1.0,  1.0,  1.0};
double init_K_v_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};

// ##############################
// Constants

const int r_id = 0;

using namespace std;
using namespace Eigen;
using namespace std;
using namespace kinematics;
using namespace dynamics;

// ##############################
// Helper Functions
void wrenchToJointVels (const Vector6d& wrench, Vector7d& dq, bool useLeftArm) {

	// Get the Jacobian towards computing joint-space velocities
	const char* nodeName = useLeftArm ? "lGripper" : "rGripper";
	static kinematics::BodyNode* eeNode = world->getSkeleton(r_id)->getNode(nodeName);
	MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Compute the inverse of the Jacobian
	Eigen::MatrixXd Jt = J.transpose();
	Eigen::MatrixXd Jinv = Jt * (J * Jt).inverse();

	// Get the joint-space velocities by multiplying inverse Jacobian with the opposite wrench.
	dq = Jinv * wrench / 300.0;
	//pv(dq);

	// Threshold the velocities
	for(size_t i = 0; i < 7; i++) {
		if(dq(i) > 0.1) dq(i) = 0.2;
		else if(dq(i) < -0.1) dq(i) = -0.2;
	}
}

void do_init_pids(somatic_motor_t* mot, pid_state_t* pids) {
    for(int i = 0; i < mot->n; i++) {
        pids[i].pos_target = 0.0;
        pids[i].vel_target = 0.0;
        pids[i].pos_error_last = 0.0;
        pids[i].vel_error_last = 0.0;
        for(int j = 0; j < PID_ERROR_WINDOW_SIZE; j++) pids[i].pos_error_window[j] = 0.0;
        for(int j = 0; j < PID_ERROR_WINDOW_SIZE; j++) pids[i].vel_error_window[j] = 0.0;
        pids[i].pos_target = mot->pos[i];
    }
}

void update_pids(somatic_motor_t* mot, pid_state_t* pids, double* result) {
    double p_p_value;
    double p_d_value;
    double v_p_value;
    double v_d_value;

    double pos_error;
    double vel_error;

    for(int i = 0; i < mot->n; i++) {
        result[i] = 0;

        if(pids[i].use_pos) {
            pos_error = pids[i].pos_target - mot->pos[i];

            p_p_value = pids[i].K_p_p * pos_error;
            p_d_value = pids[i].K_p_d * (pos_error - pids[i].pos_error_last);

            result[i] += p_p_value + p_d_value;

            pids[i].pos_error_last = pos_error;
        }
        if (pids[i].use_vel) {
            vel_error = pids[i].vel_target - mot->vel[i];

            v_p_value = pids[i].K_v_p * vel_error;
            v_d_value = pids[i].K_v_d * (vel_error - pids[i].vel_error_last);

            result[i] += v_p_value + v_d_value;

            pids[i].vel_error_last = vel_error;
        }
    }
}


vector <int> arm_ids;		///< The index vector to set config of arms
vector <int> imuWaist_ids; ///< The index vector to set config of waist/imu 

/* ******************************************************************************************** */
void computeExternal (double imu, double waist, const somatic_motor_t& lwa, const Vector6d& 
		input, SkeletonDynamics& robot, Vector6d& external, bool left) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame by setting the arm values
	// and the imu/waist values
	robot.setConfig(arm_ids, Map <Vector7d> (lwa.pos));
	robot.setConfig(imuWaist_ids, Vector2d(imu, waist));
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d Rsw = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();
	vector <int> dofs;
	for(size_t i = 0; i < 24; i++) dofs.push_back(i);
	//cout << "\nq in computeExternal: " << robot.getConfig(dofs).transpose() << endl;
	
	// Create the wrench with computed rotation to change the frame from the world to the sensor
	Matrix6d pSsensor_world = MatrixXd::Identity(6,6); 
	pSsensor_world.topLeftCorner<3,3>() = Rsw;
	pSsensor_world.bottomRightCorner<3,3>() = Rsw;
	
	// Get the weight vector (note that we use the world frame for gravity so towards -y)
	// static const double eeMass = 0.169;	// kg - ft extension
	Vector6d weightVector_in_world;
	weightVector_in_world << 0.0, 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d wrenchWeight = pTcom_sensor * pSsensor_world * weightVector_in_world;

	// Remove the effect from the sensor value and convert the wrench into the world frame
	external = input - wrenchWeight;
	external = pSsensor_world.transpose() * external;	
}

/* ******************************************************************************************** */
void computeOffset (double imu, double waist, const somatic_motor_t& lwa, const Vector6d& raw, 
		    SkeletonDynamics& robot, Vector6d& offset, bool left) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame. 
	robot.setConfig(imuWaist_ids, Vector2d(imu, waist));
	robot.setConfig(arm_ids, Map <Vector7d> (lwa.pos));
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d R = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();

	// Create the wrench with computed rotation to change the frame from the bracket to the sensor
	Matrix6d pSsensor_bracket = MatrixXd::Identity(6,6); 
	pSsensor_bracket.topLeftCorner<3,3>() = R;
	pSsensor_bracket.bottomRightCorner<3,3>() = R;
	
	// Get the weight vector (note that we use the bracket frame for gravity so towards -y)
	Vector6d weightVector_in_bracket;
	weightVector_in_bracket << 0.0, 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d expectedFT = pTcom_sensor * pSsensor_bracket * weightVector_in_bracket;
	pv(raw);
	pv(expectedFT);

	// Compute the difference between the actual and expected f/t values
	offset = expectedFT - raw;
	pv(offset);
}

/* ********************************************************************************************* */
bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data) {

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &ft_chan, &numBytes, &abstimeout, 
		ACH_O_LAST, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return false;

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return false;
	if(msg->meta->type != SOMATIC__MSG_TYPE__FORCE_MOMENT) return false;

	// Read the force-torque message and write it into the vector
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(daemon_cx.pballoc), 
		numBytes, buffer);
	for(size_t i = 0; i < 3; i++) data(i) = ftMessage->force->data[i]; 
	for(size_t i = 0; i < 3; i++) data(i+3) = ftMessage->moment->data[i]; 
	return true;
}
/* ********************************************************************************************* */
// Reads waist data and returns true if data successfully updated
bool getWaist(double* waist, ach_channel_t& waistChan) {
	// Get the time to get the sensor values by
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);

	// Get the data from the motors
	int r;
	Somatic__MotorState * waistState = SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, 
		&protobuf_c_system_allocator, 1024, &waistChan, &abstime);
	
	// Ach sanity check
	aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME,
			"Ach wait failure %s on pcio data receive (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)),
			__FILE__, __LINE__);

	if (r == ACH_OK) {
		// Read the data
		*waist = waistState->position->data[0];
		
		// Free the memory
		somatic__motor_state__free_unpacked(waistState, &protobuf_c_system_allocator );
		return true;
	}

	return false;
}

/* ********************************************************************************************* */
void getImu (double *imu, ach_channel_t& imuChan) {

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Prepare the ssdmu structure 
	ssdmu_sample_t imu_sample;
	imu_sample.x  = imu_msg->data[0];
	imu_sample.y  = imu_msg->data[1];
	imu_sample.z  = imu_msg->data[2];
	imu_sample.dP = imu_msg->data[3];
	imu_sample.dQ = imu_msg->data[4];
	imu_sample.dR = imu_msg->data[5];

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	// Make the calls to extract the pitch and rate of extraction
	*imu = -ssdmu_pitch(&imu_sample) + M_PI/2;				 
}

int do_set_limits(somatic_motor_t* lwa) {
  // Set the min/max values for valid and limits values
  double** limits [] = {
    &lwa->pos_valid_min, &lwa->vel_valid_min, 
    &lwa->pos_limit_min, &lwa->vel_limit_min, 
    &lwa->pos_valid_max, &lwa->vel_valid_max, 
    &lwa->pos_limit_max, &lwa->vel_limit_max};
  for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, 7);
  for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, 7);
}

void init_ft(somatic_motor_t* lwa, Vector6d* offset){
  // Initialize the channels to the imu and waist sensors
  somatic_d_channel_open(&daemon_cx, &imu_chan, "imu-data", NULL);
  somatic_d_channel_open(&daemon_cx, &waist_chan, "waist-state", NULL);

  // Get imu data
  double imu = 0.0;
  for(int i = 0; i < 500; i++) {
    double temp;
    getImu(&temp, imu_chan);
    imu += temp;
  }
  imu /= 500;
  cout << "imu : " << imu*180.0/M_PI << endl;

  // Get waist data
  double waist;
  while(!getWaist(&waist, waist_chan));
  cout << "waist : " << waist*180.0/M_PI << endl;

  // Open the state and ft channels
  somatic_d_channel_open(&daemon_cx, &ft_chan, "llwa_ft", NULL);
  somatic_d_channel_open(&daemon_cx, &ft_chan, "rlwa_ft", NULL);

  // Get the first force-torque reading and compute the offset with it
  cout << "reading FT now" << endl;
  Vector6d ft_data, temp;
  ft_data << 0,0,0,0,0,0;
  for(size_t i = 0; i < 1e3; i++) {
    bool gotReading = false;
    while(!gotReading) 
      gotReading = getFT(daemon_cx, ft_chan, temp);
    ft_data += temp;
  }
  ft_data /= 1e3;
	
  computeOffset(imu, waist, *lwa, ft_data, *(world->getSkeleton(0)), *offset, true);
}

// main loop
int main(int argc, char* argv[]) {
  // variables
  double lmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double rmessage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  pid_state_t lpids[7];
  pid_state_t rpids[7]; 

  // Force feedback variables
  Vector6d lraw, lexternal;
  Vector7d ldq, lvals;
  vector <int> larm_ids;
  for(size_t i = 4; i < 17; i+=2) larm_ids.push_back(i + 6);

  // Vector6d rraw, rexternal;
  // Vector7d rdq, rvars;
  // vector <int> rarm_ids;
  // for(size_t i = 4; i < 17; i+=2) larm_ids.push_back(i + 6);

  double waist = 0.0, imu = 0.0;
  double dqZero [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // init daemon
  somatic_d_opts_t daemon_opt;
  memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
  daemon_opt.ident = "arm-joy-current";
  somatic_d_init(&daemon_cx, &daemon_opt);

  // init motors
  somatic_motor_init(&daemon_cx, &llwa, 7, "llwa-cmd", "llwa-state");
  somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
  somatic_motor_update(&daemon_cx, &llwa);
  somatic_motor_update(&daemon_cx, &rlwa);
  // somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
  // somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
  usleep(1e5);

  // set limits
  do_set_limits(&llwa);
  do_set_limits(&rlwa);

  // start the daemon running
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

  // start our targets at where the motors actually are
  somatic_motor_update(&daemon_cx, &llwa);
  somatic_motor_update(&daemon_cx, &rlwa);
  do_init_pids(&llwa, lpids);
  do_init_pids(&rlwa, rpids);
  for(int i = 0; i < 7; i++) {
    lpids[i].K_p_p = init_K_p_p[i];
    lpids[i].K_p_d = init_K_p_d[i];
    lpids[i].K_v_p = init_K_v_p[i];
    lpids[i].K_v_d = init_K_v_d[i];
    lpids[i].use_pos = use_pos[i];
    lpids[i].use_vel = use_vel[i];
    rpids[i].K_p_p = init_K_p_p[i];
    rpids[i].K_p_d = init_K_p_d[i];
    rpids[i].K_v_p = init_K_v_p[i];
    rpids[i].K_v_d = init_K_v_d[i];
    rpids[i].use_pos = use_pos[i];
    rpids[i].use_vel = use_vel[i];
  }


  init_ft(&llwa, &loffset);

  size_t c = 0; // counter

  // Begin the run loop
  while(!somatic_sig_received) {
    c++;
    // Update left arm kinematic model
    somatic_motor_update(&daemon_cx, &llwa);
    for(size_t i = 0; i < 7; i++) lvals(i) = llwa.pos[i];
    world->getSkeleton(r_id)->setConfig(larm_ids, lvals);

    // Update right arm kinematic model
    somatic_motor_update(&daemon_cx, &rlwa);
    // TODO: complete right arm stuff    

    getImu(&imu, imu_chan);
    getWaist(&waist, waist_chan);
    
    // Get the f/t sensor data and compute the ideal value
    size_t k = 1e1;
    bool result = (c % k == 0) && getFT(daemon_cx, ft_chan, lraw);
    if(!result) continue;

    Vector6d lideal = lraw + loffset;
    //    Vector6d rideal = rraw + roffset;


    computeExternal(imu, waist, llwa, lideal, *(world->getSkeleton(r_id)), lexternal, true);

    // Threshold the values - 4N for forces, 0.4Nm for torques
    if((lexternal.topLeftCorner<3,1>().norm() < 7) && 
       (lexternal.bottomLeftCorner<3,1>().norm() < 0.4)) {
      somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, dqZero, 7, NULL);
      continue;
    }
    
    // Increase the sensitivity to the torque values
    lexternal(3) *= 20;
    lexternal(4) *= 20;
    lexternal(5) *= 40;
    pv(lexternal);
    
    // Compute the necessary motion in the joint space by multiply the -external with Jacobian inv
    wrenchToJointVels(lexternal, ldq, true);
    pv(ldq);
    
    // do the pid control thing
    update_pids(&llwa, lpids, lmessage);
    update_pids(&rlwa, rpids, rmessage);

  }

  // Send stopping event
  somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

  // halt the motor
  somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
  somatic_motor_destroy(&daemon_cx, &rlwa);
  somatic_d_destroy(&daemon_cx);

}
