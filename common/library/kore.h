/**
 * @file kore.h
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The main header file for the "K"rang "O"peration "R"untime "E"nvironment 
 */

#pragma once

#include "sensors.h"
#include "safety.h"

using namespace Eigen;

namespace Krang {

/* ******************************************************************************************** */
/// The interface to the active motor groups on the robot.
class Hardware {
public:

	/// The indicators for the motor groups to be used
	enum Mode {
		MODE_AMC = 1,
		MODE_LARM = 2,
		MODE_RARM = 4,
		MODE_TORSO = 8,
		MODE_WAIST = 16,
		MODE_GRIPPERS = 32,													///< Indicates the robotiq grippers (default)
		MODE_GRIPPERS_SCH = 64,											///< Indicates the schunk grippers
		MODE_ALL = MODE_AMC | MODE_LARM | MODE_RARM | MODE_TORSO | MODE_WAIST | MODE_GRIPPERS,
		MODE_ALL_GRIPSCH = MODE_AMC | MODE_LARM | MODE_RARM | MODE_TORSO | MODE_WAIST | MODE_GRIPPERS_SCH
	};

	/// Initializes the interfaces to the motor groups based on the given hardware mode
	Hardware (Mode mode, somatic_d_t* daemon_cx, dynamics::SkeletonDynamics* robot); 

	/// The destructor which sends halt messages to all Schunk modules and 0-velocities to wheels
	~Hardware ();

public:
	// Updates the sensor readings and the kinematic structure

	/// Updates the sensor readings
	void updateSensors(double dt);

	/// Prints the state
	void printState();
	void printStateCurses(int row, int col);

private:	

	/// Updates the Dart's kinematics data structures with the latest readings
	/// This is made private because updateSensors already calls this. A user should not need to call
	/// it.
	void updateKinematics();

public:
	// Initializes the modules and sensors

	/// Initializes a motor group that is represented with a somatic structure
	void initMotorGroup (somatic_motor_t*& motors, const char* cmd_name, const char* state_name, 
			VectorXd minPos, VectorXd maxPos, VectorXd minVel, VectorXd maxVel);

	/// Initializes the amc wheels while using the average imu to create an offset (for absolute 
	/// wheel positions)
	void initWheels ();

	/// Initializes the imu channel, the filter and averages first 500 readings for correct 
	/// wheels and f/t offsets
	void initImu ();

	/// Initializes the waist module group: (1) first creates the somatic interface without a command
	/// channel to get updates, (2) second, creates a channel to the waist daemon
	void initWaist ();

public:
	// The fields to keep track of the daemon context, mode and the robot kinematics
	
	Mode mode;														///< Indicates which motor groups are used
	somatic_d_t* daemon_cx;								///< The daemon context for the running program
	dynamics::SkeletonDynamics* robot;		///< The kinematics of the robot

public:
	// The interfaces to the sensors on Krang

	ach_channel_t* imu_chan;							///< Imu sensor for balance angle
	double imu, imuSpeed;									///< Latest imu readings (or the mean over a window)
	filter_kalman_t* kfImu; 							///< The kalman filter for the imu readings

	FT* lft;															///< Force/torque data from left arm
	FT* rft;															///< Force/torque data from right arm

public:
	// The interfaces to the motor groups on Krang

	somatic_motor_t* amc;									///< Wheel motors interface
	somatic_motor_t* larm;								///< Left arm motors interface
	somatic_motor_t* rarm;								///< Right arm motors interface
	somatic_motor_t* torso;								///< Torso motor interface
	somatic_motor_t* grippers;						///< Gripper motors interface
	somatic_motor_t* waist;								///< Waist motors interface - only read
	ach_channel_t* waistCmdChan;					///< Command channel for the waist daemon
};

};	// end of namespace
