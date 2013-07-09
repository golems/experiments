/**
 * @file Dynamics.h
 * @author Can Erdogan
 * @date Feb 12, 2013
 * @brief Performs center of mass computations.
 * NOTE We refer to 10 sections of Krang in the comments. These sections are: 
 *   1) base: upto and including waist modules (no wheels)
 *   2) spine: upto and including the torso module 
 *   3) shoulder bracket: including Kinect
 *   4-9) the seven links on an arm. 
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

#define NUM_SECTIONS 10

/* ******************************************************************************************** */
/// The class for dynamics modeling
class Dynamics {
public:

	/// Define the data read for a part
	struct Part {
		char name [5];							///< The name of the part
		size_t section;							///< The section the part belongs to
		size_t used;								///< 1 if the part should be used in CoM calculation
		double mass;								///< The mass of the part
		Eigen::Vector3d comPos;			///< The position of the CoM wrt the part frame
		Eigen::Vector3d sectionPos;	///< The position of the part frame wrt the section frame
	};

	Eigen::Vector4d sectionCoMs [NUM_SECTIONS];	///< Center of masses of the 5 major sections of Krang
	double sectionMasses [NUM_SECTIONS];				///< Masses of each section to use when combining them

	/// The constructor that reads in the mass table and computes the com's of the 5
	/// main sections in their local frames.
	// NOTE The fix alignment variable is added to test the unaligned values.
	Dynamics (const char* massTablePath, bool fixAlignment = true);

	/// Changes the section CoMs so they are in the DH coord. frames rather than hardware aligned
	/// frames.
	void moveReferenceFrames();

	/// CoM of the robot given its configuration in homogeneous coordinates of the world frame
	Eigen::Vector3d com (double q_imu, double q_w, double q_tor, const Eigen::VectorXd& leftArm, 
		const Eigen::VectorXd& rightArm);

	/// CoM of an arm given its configuration in the base frame
	Eigen::Vector4d armCoM (const Eigen::VectorXd& qs, double& totalMass);

	/// Parses the mass table
	static void parse(const char* filePath, std::vector <Part>& parts);
};
