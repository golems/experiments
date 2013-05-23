/**
 * @file Kinematics.h
 * @author Can Erdogan
 * @date Feb 13, 2013
 * @brief Contains the DH parameterization and a set of functions to retrieve the transformations
 * between some modules for convenience and testing.
 * NOTE We are setting the mechanical constants such as link lengths here as we expect that whenever
 * they are needed, this file can be included.
 */

#pragma once

#include <Eigen/Dense>

/* ******************************************************************************************** */
/// The static class for transformations and fixed constants 
namespace kinematics {

	// ======================================================================
	// The fixed mechanical constants (in cms or radians)
	// NOTE This values are obtained from the solidworks model which does not
	// incorporate any deformations of the hardware

	/// The fixed angle difference between the base to the line between wheel axis and the waist
  /// modules, \theta^o_1 in docs.
	static const double theta1Offset = 0.1838;		

	static const double L1 = 50.74;				///> Distance between wheel axes and the waist module
	static const double L2 = 10.98;				///> Distance between the waist module and the spine
	static const double L3 = 66.94;				///> Distance between the torso frame's origin and the bases of the arms
	static const double L4 = 28.06;				///> Arm link length between modules 0 and 2
	static const double L5 = 32.78;				///> Arm link length between modules 2 and 4
	static const double L6 = 27.63;				///> Arm link length between modules 4 and 6
	static const double L7 = 17.07;				///> Arm link length between modules 6 and 7
	static const double L8 = 15.1;					///> Dist b/w end of module 7 and start of e-e (black bottom surface)

	// ======================================================================
	// Transformation functions

	/// Computes a transformation given the DH parameters
	static inline Eigen::Matrix4d dh (double a, double alpha, double d, double theta) {

		// Compute the cos and sin functions of the input degrees
		double cth = cos(theta), sth = sin(theta);
		double calp = cos(alpha), salp = sin(alpha);

		// Set the matrix
		Eigen::Matrix4d T;
		T << 
			cth, -sth * calp,  sth * salp, a * cth,
			sth,  cth * calp, -cth * salp, a * sth,
			0.0,        salp,        calp,       d,
			0.0,         0.0,         0.0,     1.0;  

		return T;
	}

	/// Waist frame in world
	static inline Eigen::Matrix4d waist (double q_imu) {
		return dh(L1, 0.0, 0.0, (M_PI_2 - q_imu - theta1Offset));
	}								

	/// Torso frame in world
	static inline Eigen::Matrix4d torso (double q_imu, double q_w) {
		return waist(q_imu) * dh(L2, -M_PI_2, 0.0, (M_PI_2 - q_w + theta1Offset)); 
	}

	/// Left arm base frame in world
	static inline Eigen::Matrix4d leftArmBase (double q_imu, double q_w, double q_tor) {
		return torso(q_imu, q_w) * dh(0.0, -M_PI_2, -L3, M_PI + q_tor);
	}

	/// Right arm base frame in world
	static inline Eigen::Matrix4d rightArmBase (double q_imu, double q_w, double q_tor) {
		return torso(q_imu, q_w) * dh(0.0, -M_PI_2, -L3, q_tor);
	}

	/// L4 frame in the base frame of the corresponding arm base
	static inline Eigen::Matrix4d l4 (const Eigen::VectorXd& qs) {
		return dh(0.0, -M_PI_2, -L4, qs(0));
	}

	/// L5 frame in the base frame of the corresponding arm base
	static inline Eigen::Matrix4d l5 (const Eigen::VectorXd& qs) {
		return l4(qs) * dh(0.0, M_PI_2, 0.0, qs(1));
	}

	/// L6 frame in the base frame of the corresponding arm base
	static inline Eigen::Matrix4d l6 (const Eigen::VectorXd& qs) {
		return l5(qs) * dh(0.0, -M_PI_2, -L5, qs(2));
	}

	/// L7 frame in the base frame of the corresponding arm base
	static inline Eigen::Matrix4d l7 (const Eigen::VectorXd& qs) {
		return l6(qs) * dh(0.0, M_PI_2, 0.0, qs(3));
	}

	/// L8 frame in the base frame of the corresponding arm base
	static inline Eigen::Matrix4d l8 (const Eigen::VectorXd& qs) {
		return l7(qs) * dh(0.0, -M_PI_2, -L6, qs(4));
	}

	/// L9 frame in the base frame of the corresponding arm base
	static inline Eigen::Matrix4d l9 (const Eigen::VectorXd& qs) {
		return l8(qs) * dh(0.0, M_PI_2, 0.0, qs(5));
	}

	/// L10 frame in the base frame of the corresponding arm
	/*  The origin of the frame of reference here is assumed to be at the center of
   * the bottom black surface of End Effector. We take that as the origin
   * because that is the origin of the frame in which the end-effector's COM is
   * defined in the robotiq manual. The XYZ axes are parallel to the previous
   * frame at home position of Motor-7 and the two fingers are equidistant from 
   * the YZ plane while the third finger is cut through by the YZ plane.
	 */
	static inline Eigen::Matrix4d l10 (const Eigen::VectorXd& qs) {	 
		return l9(qs) * dh(0.0, 0.0, -L7-L8, qs(6));
	}
};


