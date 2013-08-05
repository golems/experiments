/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file 01-massTable.cpp
 * @author Can Erdogan
 * @date July 10, 2013
 * @brief This file shows the evaluation of the mass table for Krang. The idea is that we have
 * detailed measurements of most of the parts of Krang but these parts can only be represented as 
 * assembled nodes in dart with urdf files. Given that an assembly (node) can have up to 20 parts, 
 * we would like to compute the center of mass (com) of the node using the com, the mass and the
 * location wrt to the node frame of the individual parts. 
 * The output of this exe is just the node's coms and masses. The values might not be correctly
 * aligned with the dart frames so pay attention while placing them in urdf files. 02-com should
 * let you visualize them.
 * Note we use the words node/assembly/section interchangeably.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

/// Define the data read for a part
struct Part {
	char name [5];							///< The name of the part
	size_t section;							///< The section the part belongs to
	size_t used;								///< 1 if the part should be used in CoM calculation
	double mass;								///< The mass of the part
	Eigen::Vector3d comPos;			///< The position of the CoM wrt the part frame
	Eigen::Vector3d sectionPos;	///< The position of the part frame wrt the section frame
};

/* ******************************************************************************************** */
/// Parses the mass table at the given path and creates the parts with mass/com/location info
void parse(const char* filePath, std::vector <Part>& parts){

	// Open file for read access and if it fails, return -1
	FILE *file = fopen(filePath, "r");
	if(file == NULL) {
		printf("File path: '%s'\n", filePath);
		assert(false && "Could not open the file.");
	}

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

/* ******************************************************************************************** */
/// The main thread
int main () {

	// ===========================================================================
	// Parse the table

	vector <Part> parts;
	parse(MASS_TABLE_PATH, parts);

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
			sectionCoM /= totalMass;
			cout << "section " << p.section << " - " << p.name << " : mass:" << totalMass << 
				", com: " << sectionCoM.transpose() << endl;

			// Reset the accumulating variables and set the new section 
			sectionCoM = Vector4d::Zero();
			totalMass = 0.0;
			sectionIdx++;
		}
	}
}

