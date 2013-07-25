/**
 * @file sensors.h
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The header file for the sensor class and helper functions associated with Krang.
 */

#pragma once

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <filter.h>
#include <ach.h>

#include <Eigen/Dense>
#include <dynamics/SkeletonDynamics.h>

using namespace dynamics;

typedef Eigen::Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Eigen::Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain f/t values
#define eig7(x) (Vector7d() << (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5], (x)[6]).finished()

extern std::vector <int> left_arm_ids;			///< Ids for left arm
extern std::vector <int> right_arm_ids;			///< Ids for right arm
extern std::vector <int> imuWaist_ids;			///< Ids for waist/imu

namespace Krang {

/// The indicator for the left or right side	
enum Side {
        LEFT = 0,
        RIGHT
};

/* ******************************************************************************************** */
class FT {
public:

	/// Indicates which type of gripper is after the f/t sensor which affects the mass and com
	enum GripperType {
		GRIPPER_TYPE_ROBOTIQ = 0,
		GRIPPER_TYPE_SCHUNK,
		GRIPPER_TYPE_NONE
	};

	/// The constructor. The type of the gripper after f/t affects the readings, robot + side are
	/// used to estimate the frame of the f/t in the robot base frame.
	FT (GripperType type, somatic_d_t* daemon_cx, dynamics::SkeletonDynamics* robot, Side side);

        /// Gets the latest raw reading from the f/t channel
        bool getRaw(Vector6d& raw) {}

        /// Updates the estimate for the external f/t values
        void update() {}

public:
	// Variables to compensate for the weight of the gripper and the data channel

        Vector6d external;///< The most recent estimate of the external force/torque input
	/// The offset from the raw readings for the ideal readings. An ideal reading still has the
	/// weight of the gripper and any other external weights in it.
	Vector6d offset;	
	double gripperMass;					///< The mass of the objects after the f/t sensor
        Eigen::Vector3d gripperCoM;				///< The center of mass of the objects after the f/t sensor
	ach_channel_t* chan;				///< The data channel

public:
	// Variables to determine the kinematics that affect the compensation

	Side side;															///< Indices the left or the right f/t sensor
	dynamics::SkeletonDynamics* robot;			///< The kinematics of the robot
        // TODO SET this
        somatic_d_t* daemon_cx;
};

/* ******************************************************************************************** */
// Helper functions to update other sensors (i.e. imu, kinect..)

/// Returns the imu value and filters it if a filter struct is given
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
             filter_kalman_t* kf);

};
