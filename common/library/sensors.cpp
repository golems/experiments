/**
 * @file sensors.cpp
 * @author Can Erdogan
 * @date July 24, 2013
 * @brief The source file for the sensor class and helper functions associated with Krang.
 */

#include "sensors.h"


/* ******************************************************************************************** */
FT::FT (GripperType type, dynamics::SkeletonDynamics* robot, Side side) {

	// Sanity check the input
	assert((robot != NULL) && "Can not initialize the f/t sensors without robot kinematics");

	// Open the data channel
	somatic_d_channel_open(&daemon_cx, &chan, (side == LEFT) ? "llwa_ft" : "rlwa_ft", NULL);

	// Set the gripper mass and center of mass based on the type: the com for ext, schunk and robotiq
	// are 0.026, 0.0683, 0.065 (?)
	gripperMass = 0.169;			//< f/t extension mass (will be there regardless of type)
	if(type == GRIPPER_TYPE_ROBOTIQ) {
		gripperMass += 2.3;
		gripperCoM = Vector3d(0.0, 0.0, 0.09);
	}
	else if (type == GRIPPER_TYPE_SCHUNK) {
		gripperMass += 1.6;
		gripperCoM = Vector3d(0.0, -0.008, 0.091);
	}
	else gripperCoM = Vector3d(0.0, -0.008, 0.013);

	// Average the first 1000 readings to compute the offset
	Vector6d ft_data = Vector6d::Zero(), temp;
	for(size_t i = 0; i < 1e3; i++) {
		bool gotReading = false;
		while(!gotReading) gotReading = getFT(daemon_cx, ft_chan, temp);
		ft_data += temp;
	}
	ft_data /= 1e3;

	// Compute the offset between what the reading should be and what it is assuming no external
	// forces
//	error(reading, offset);	
}

/* ******************************************************************************************** */
