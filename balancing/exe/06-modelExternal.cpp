/**
 * @file 06-modelExternal.cpp
 * @author Munzir Zafar
 * @date July 14 2013
 * @brief This file gives the effect of wrench at the end-effector on the wheels of the robot
 */

/* ******************************************************************************************* */
// Compute the wrench on the wheel as an effect of t he wrench acting on the sensor

void computeWheelWrench(Vector3d& wh2s, Vector6d& wrenchSensor, Vector6d& wrenchWheel) {
	
	// Get the wrench shift operator to move the wrench from sensor origin to the wheel axis
	Matric6d pTsensor_wheel = MatrixXd::Identity(6,6);
	pTsensor_wheel.bottomLeftCorner<3,3>() << 0.0, -wh2s(2), wh2s(1), whe2s(2), 0.0, -wh2s(0),
		-wh2s(1), wh2s(0), 0.0;

	// Shift the wrench from the sensor origin to the wheel axis
	Vector6d wrenchWheel = pTsensor_wheel * wrenchSensor;
	
}



/* ******************************************************************************************** */
/// The main thread
int main(const int argc, char** argv) {

	return 0;
}
