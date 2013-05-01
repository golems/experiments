/**
 * @file Dynamics.h
 * @author Can Erdogan
 * @date Feb 12, 2013
 * @brief Performs center of mass computations.
 */

#include "Dynamics.h"
#include "Kinematics.h"
#include <stdio.h>
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace kinematics;

/* ******************************************************************************************** */
Dynamics::Dynamics (const char* massTablePath, bool fixAlignment) {

	// ===========================================================================
	// Parse the table
	vector <Part> parts;
	parse(massTablePath, parts);

	// Compute the CoM's of each section
	size_t sectionIdx = 0, numParts = parts.size();
	double totalMass = 0.0;
	Vector4d sectionCoM = Vector4d::Zero();
	for(size_t i = 0; i < numParts; i++) {

		// Get the CoM of the part in the section frame
		const Part& p = parts[i];
		Vector4d partCoM = (p.comPos + p.sectionPos).homogeneous();

		// Accumulate the CoM if the object should be included in the calculation
		// NOTE We do not "continue" here because even though the object might be ignored,
		// because it is the last one, at this index, we might need to finish computation
		if(p.used == 1) {
			sectionCoM += p.mass * partCoM;
			totalMass += p.mass;
		}

		// If the next part is in another section or this is the last part, complete calculation
		if((i == (numParts - 1)) || (parts[i+1].section != p.section)) {

			// Finish the computation of the previous section
			assert((totalMass != 0.0) && "The total mass can not be 0.0!");
			sectionCoMs[sectionIdx] = sectionCoM / totalMass;
			sectionMasses[sectionIdx] = totalMass;
			// cout << "section " << p.section << ": mass:" << totalMass << ", com: " << sectionCoM/totalMass << endl;

			// Reset the accumulating variables and set the new section 
			sectionCoM = Vector4d::Zero();
			totalMass = 0.0;
			sectionIdx++;
		}
	}

	// Align the axes with the DH frames rather than hardware aligned coordinates
	if(fixAlignment) moveReferenceFrames();
}

/* ******************************************************************************************** */
void Dynamics::moveReferenceFrames() {

	bool debug = 0;

	// ===========================================================================
	// Convert the CoMs of the sections from their hardware-aligned frames (hf's) 
	// to the DH frames (df's): see CoM computation document
	
	// Convert frame 1: the transformation from hf1 to df1 can be decomposed into
	// three simpler transformations: first Rot(x2,90), second Rot(z2, 90-offset),
	// and lastly Trans(x2,-L1). 
	Matrix3d rotX, rotY, rotZ;
	rotX = AngleAxisd(-M_PI_2, Vector3d::UnitX());
	rotZ = AngleAxisd(-(M_PI_2 - theta1Offset), Vector3d::UnitZ()); 
	Vector3d rotated1 = rotX * sectionCoMs[0].head<3>();
	Vector3d rotated2 = rotZ * rotated1;
	if(debug) cout << "rotated1: " << rotated1.transpose() << endl;
	if(debug) cout << "rotated2: " << rotated2.transpose() << endl;
	sectionCoMs[0] = (rotated2 + Vector3d(-L1, 0, 0)).homogeneous(); 

	// Convert frame 2: the t. from hf2 to df1 can be decomposed as first a rotation
	// around y axis for 180 degrees and then a translation along x for L2.
	rotY = AngleAxisd(M_PI, Vector3d::UnitY());
	Vector3d rotated = rotY * sectionCoMs[1].head<3>();
	sectionCoMs[1] = (rotated + Vector3d(-L2, 0, 0)).homogeneous(); 
}

/* ******************************************************************************************** */
Vector3d Dynamics::com (double q_imu, double q_w, double q_tor, const VectorXd& lqs, 
		const VectorXd& rqs) {

	bool debug = 0;

	// =================================================================
	// Get the center of masses of each section

	// Get the base
	Matrix4d baseT = waist(q_imu);
	Vector4d base = baseT * sectionCoMs[0];

	// Get the spine
	Matrix4d spineT = torso(q_imu, q_w);
	Vector4d spine = spineT * sectionCoMs[1];

	// Get the shoulder bracket
	Matrix4d shoulderT = leftArmBase(q_imu, q_w, q_tor);
	Vector4d shoulder = shoulderT * sectionCoMs[2];

	// Get the left arm CoM
	double leftArmMass = 0.0;
	Matrix4d leftArmT = leftArmBase(q_imu, q_w, q_tor);
	Vector4d leftArm = leftArmT * armCoM(lqs, leftArmMass);
	
	// Get the right arm CoM
	double rightArmMass = 0.0;
	Matrix4d rightArmT = rightArmBase(q_imu, q_w, q_tor);
	Vector4d rightArm = rightArmT * armCoM(rqs, rightArmMass);

	// =================================================================
	// Combine the center of masses

	// Compute the upper body CoM	
	bool onlyLowerBody = true;
	Vector4d upperBodyCoMNumerator = sectionMasses[2] * shoulder + leftArmMass * leftArm + rightArmMass * rightArm;
	double upperBodyMass = onlyLowerBody ? 0.0 : (sectionMasses[2] + leftArmMass + rightArmMass);

	// Compute the total CoM
	Vector4d result = sectionMasses[0] * base + sectionMasses[1] * spine + (onlyLowerBody ? Vector4d(0.0, 0.0, 0.0, 0.0) : upperBodyCoMNumerator);
	double totalMass = sectionMasses[0] + sectionMasses[1] + upperBodyMass;
	return (result / totalMass).head<3>();
}

/* ******************************************************************************************** */
Eigen::Vector4d Dynamics::armCoM (const Eigen::VectorXd& qs, double& totalMass) {

	size_t endEffectorPresent = 0;

	// Move all the arm section CoMs to the base frame
	Matrix4d Ts [7] = {l4(qs), l5(qs), l6(qs), l7(qs), l8(qs), l9(qs), l10(qs)};
	Vector4d result = Vector4d::Zero();
	totalMass = 0.0;
	for(size_t i = 0; i < 6+endEffectorPresent; i++) {
		result += sectionMasses[3+i] * Ts[i] * sectionCoMs[3+i];
		totalMass += sectionMasses[3+i];
	}

	// Take the mean of the weighted center of masses
	return (result / totalMass);
}

/* ******************************************************************************************** */
void Dynamics::parse(const char* filePath, std::vector <Part>& parts){

	// Open file for read access and if it fails, return -1
	FILE *file = fopen(filePath, "r");
	assert((file != NULL) && "Could not open the file.");

	// Read in each non-commented line of the config file corresponding to each joint
	int lineNum = 0;
	char line[1024];
	static const size_t kNumParams = 10;
	while (fgets(line, sizeof(line), file) != NULL) {
		
		// Skip the line if it is commented - check the first character
		if(line[0] == '#') continue;

		// Get the parameters
		Part p;
		size_t result = sscanf(line, "%s%lu%lu%lf%lf%lf%lf%lf%lf%lf", p.name, &p.section, &p.used, 
			&p.mass, &p.comPos(0), &p.comPos(1), &p.comPos(2), &p.sectionPos(0), &p.sectionPos(1),	
			&p.sectionPos(2));
		assert((result == kNumParams) && "Could not parse a line");

		// Increment the line counter and populate the output vector
		parts.push_back(p);
		lineNum++; 
	}

	// Close the file 
	fclose(file); 
}
