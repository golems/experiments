/**
 * @file 03-fr.cpp
 * @author Can Erdogan
 * @date May 19, 2013
 * @brief This executable demonstrates the first-order retraction method for task constrained
 * motion planning for Krang, trying to open a valve.
 * Some of the examples are for the pedestal and some for Krang on the ground. Look for _ground
 * extension to the function names.
 */

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include "fr.h"

using namespace simulation;
using namespace dynamics;
using namespace kinematics;
using namespace Eigen;
using namespace std;

size_t krang_id = 0;
vector <int> left_idx;
vector <int> imuWaist_ids; 
World* world;

fr* planner;

/* ********************************************************************************************** */
/// Performs forward kinematics and returns the position and orientation (quaternion) of the
/// left end-effector
void forward (const Vector7d& node, Vector3d& eePos, Quaternion <double>& eeOri) {
	world->getSkeleton(krang_id)->setConfig(left_idx, node);
	world->getSkeleton(krang_id)->setConfig(imuWaist_ids, Vector2d(1.857-3.0 * M_PI_2, (5.0 / 6) * M_PI));
	kinematics::BodyNode* eeNode = world->getSkeleton(krang_id)->getNode("lGripper");
	MatrixXd eeTransform = eeNode->getWorldTransform();
	eePos = eeTransform.topRightCorner<3,1>();
	eeOri = Quaternion <double> (eeTransform.topLeftCorner<3,3>());
	planner->leftFrame = eeTransform;
}

/* ********************************************************************************************** */
/// The task error function to follow a line with y = 0.405 and z = 1.54022 constraints
void line_err (const Vector7d& node, Vector6d& error) {

	// Perform forward kinematics
	static Vector3d eePos;
	static Quaternion <double> eeOri;
	forward(node, eePos, eeOri);

	// Get the position error
	static const Vector3d constPos (0.0, 0.405100, 1.54022);
	VectorXd errPos = constPos - eePos;

	// Set the total error with the x-axis being free
	Vector3d errOri (0.0, 0.0, 0.0);
	error << errPos, errOri;
	error(0) = 0.0;
}

/* ********************************************************************************************** */
/// The task error function to follow a circle centered at (0.48922, 0.50516) and radius 0.10 
/// with an orientation constraint. z value is 1.54013.
/// Given a node, first performs forward kinematics and then projects the end-effector position X
/// to the closest point X' on the circle. The error is |X-X'|. 
void circle_err (const Vector7d& node, Vector6d& error) {

	// Perform forward kinematics
	static Vector3d eePos;
	static Quaternion <double> eeOri;
	forward(node, eePos, eeOri);

	// The xy error is computed by assuming the point is already on the correct z value and then
	// projecting it to the circle (we get the angle wrt to the center and move radius away from it)
	double angle = atan2(eePos(1) - 0.50516, eePos(0) - 0.48922);
	double rx = 0.10 * cos(angle) + 0.48922, ry = 0.10 * sin(angle) + 0.50516;
	Vector3d errPos = Vector3d(rx, ry, 1.54013) - eePos;
	
	// Get the orientation constraint
	Vector3d constRPY (-1.57, 0.0, 1.57);
	Matrix3d constOriM = dart_math::eulerToMatrix(constRPY, dart_math::XYZ);
	Quaternion <double> constOri (constOriM); 
 
	// Find the orientation error and express it in RPY representation
	Quaternion <double> errOriQ = constOri * eeOri.inverse();
	Matrix3d errOriM = errOriM = errOriQ.matrix();
	Vector3d errOri = dart_math::matrixToEuler(errOriM, dart_math::XYZ);

	// Set the total error with the x-axis being free
	error << errPos, errOri;
}

/* ********************************************************************************************** */
/// The task error function to limit the end-effector to an yz plane with x = 0.55.
void plane_err (const Vector7d& node, Vector6d& error) {

	// Perform forward kinematics
	static Vector3d eePos;
	static Quaternion <double> eeOri;
	forward(node, eePos, eeOri);

	// Get the position error
	static const Vector3d constPos (0.440, 0.0, 0.0);
	VectorXd errPos = constPos - eePos;

	// Set the total error with the x-axis being free
	Vector3d errOri (0.0, 0.0, 0.0);
	error << errPos, errOri;
	error(1) = error(2) = 0.0;
	// pv(error);
}

/* ********************************************************************************************** */
/// The left end-effector should move upwards in the z axis while keeping the same xy values
/// and the orientation; robot is sitting on the ground.
void line_err_ground (const Vector7d& node, Vector6d& error) {

	// Perform forward kinematics
	static Vector3d eePos;
	static Quaternion <double> eeOri;
	forward(node, eePos, eeOri);

	// We want the ee to follow a line with x = 0.34 and y = 0.28.
	Vector3d errPos = Vector3d(0.3419, 0.2797, 0.0) - eePos;
	errPos(2) = 0.0;
	
	// Get the orientation constraint
	Vector3d constRPY (0.0, M_PI_2, 0.0);
	Matrix3d constOriM = dart_math::eulerToMatrix(constRPY, dart_math::XYZ);
	Quaternion <double> constOri (constOriM); 
 
	// Find the orientation error and express it in RPY representation
	Quaternion <double> errOriQ = constOri * eeOri.inverse();
	Matrix3d errOriM = errOriM = errOriQ.matrix();
	Vector3d errOri = dart_math::matrixToEuler(errOriM, dart_math::XYZ);

	// Set the total error with the x-axis being free
	error << errPos, errOri; 
//	pv(error);
}

/* ********************************************************************************************** */
/// The error function for task-constrained planning of the left arm when trying to move a box on
/// the ground using a lever. The lever remains in yz plane of the world. The fulcrum is a point
/// (fx, fy, fz), the distance of the left arm from the fulcrum is 'd' and lever is aligned along
/// -ve y-axis of the left arm
void boxLift_err_ground (const Vector7d& node, Vector6d& error) {

	// Perform forward kinematics
	static Vector3d eePos;
	static Quaternion <double> eeOri;
	forward(node, eePos, eeOri);

	// The yz error is computed by assuming the point is already on the correct x value and then
	// projecting it to the circle (we get the angle wrt to the center and move radius away from it)
	double fx = 0.252336, fy = 0.15, fz = 0.50, d = 0.20;
	double angle = atan2(eePos(2) - fz, eePos(1) - fy);
	double ry = d * cos(angle) + fy, rz = d * sin(angle) + fz;
	Vector3d errPos = Vector3d(fx, ry, rz) - eePos;
	
	// Get the orientation constraint
	Vector3d constRPY (-M_PI_2, angle-M_PI_2, -M_PI_2);
	Matrix3d constOriM = dart_math::eulerToMatrix(constRPY, dart_math::XYZ);
	Quaternion <double> constOri (constOriM); 
 
	// Find the orientation error and express it in RPY representation
	Quaternion <double> errOriQ = constOri * eeOri.inverse();
	Matrix3d errOriM = errOriM = errOriQ.matrix();
	Vector3d errOri = dart_math::matrixToEuler(errOriM, dart_math::XYZ);

	// Set the total error with the x-axis being free
	error << errPos, errOri; 
}

/* ********************************************************************************************** */
/// The left end-effector should move in a circle around the point (0.25, 0.0, 0.78)
void circle_err_ground (const Vector7d& node, Vector6d& error) {

	// Perform forward kinematics
	static Vector3d eePos;
	static Quaternion <double> eeOri;
	forward(node, eePos, eeOri);

	// The xy error is computed by assuming the point is already on the correct z value and then
	// projecting it to the circle (we get the angle wrt to the center and move radius away from it)
	double cx = 0.25, cy = 0.0, L = 0.20;
	double angle = atan2(eePos(1) - cy, eePos(0) - cx);
	double rx = L * cos(angle) + cx, ry = L * sin(angle) + cy;
	Vector3d errPos = Vector3d(rx, ry, 0.70) - eePos;
	
	// Get the orientation constraint
	Vector3d constRPY (M_PI, 0.0, angle - M_PI_2);
	Matrix3d constOriM = dart_math::eulerToMatrix(constRPY, dart_math::XYZ);
	Quaternion <double> constOri (constOriM); 
 
	// Find the orientation error and express it in RPY representation
	Quaternion <double> errOriQ = constOri * eeOri.inverse();
	Matrix3d errOriM = errOriM = errOriQ.matrix();
	Vector3d errOri = dart_math::matrixToEuler(errOriM, dart_math::XYZ);

	// Set the total error with the x-axis being free
	error << errPos, errOri; 
}

/* ********************************************************************************************** */
/// Given that the left hand is holding a stick perpendicular to its x axis and extending to its
/// y axis, and the right hand would hold it again perpendicular to its x axis but extending
/// to its -y axis, returns where the right hand should be.
void stick_constraint (const Matrix4d& left, Matrix4d& right) {
	right = left;
	right.block<4,1>(0,0) *= -1;
	right.block<4,1>(0,2) *= -1;
	right.topRightCorner<4,1>() += left.block<4,1>(0,2).normalized() * L8_Schunk;
	right.topRightCorner<4,1>() -= left.block<4,1>(0,1).normalized() * 0.56;
}

/* ********************************************************************************************** */
/// The wrench constraint - basically we are holding two sticks around an object with the
/// hands at the same orientation but symmetric around the object
void wrench_constraint (const Matrix4d& left, Matrix4d& right) {
	right = left;
	right.block<4,1>(0,0) *= -1;
	right.block<4,1>(0,2) *= -1;
	right.topRightCorner<4,1>() += left.block<4,1>(0,2).normalized() * L8_Schunk;
	right.topRightCorner<4,1>() += left.block<4,1>(0,1).normalized() * 0.40;
}

/* ********************************************************************************************** */
/// The wrench constraint - basically we are holding two sticks around an object with the
/// hands at the same orientation but symmetric around the object
void lever_constraint (const Matrix4d& left, Matrix4d& right) {
	right = left;
	right.block<4,1>(0,0) *= -1;
	right.block<4,1>(0,2) *= -1;
	right.topRightCorner<4,1>() += left.block<4,1>(0,2).normalized() * L8_Schunk;
	right.topRightCorner<4,1>() -= left.block<4,1>(0,1).normalized() * 0.20;
}

/* ********************************************************************************************** */
/// Given the joint values for the left arm and the phi value for the right arm, attempts to 
/// I.K. for the right arm with the hardcoded constraint between the left and right hands
bool computeIK (Node& node, double phi) {

	// Perform forward kinematics and determine where the right frame should be
	world->getSkeleton(krang_id)->setConfig(left_idx, node.left);
	world->getSkeleton(krang_id)->setConfig(imuWaist_ids, Vector2d(1.857-3.0 * M_PI_2, (5.0 / 6) * M_PI));
	Matrix4d leftFrame = world->getSkeleton(krang_id)->getNode("lGripper")->getWorldTransform(), 
		rightFrame;

	pv(node.left);
	pmr(leftFrame);
	stick_constraint(leftFrame, rightFrame);
	pmr(rightFrame);

	// Get the relative goal
	static Transform <double, 3, Affine> relGoal;
	getWristInShoulder(world->getSkeleton(krang_id), rightFrame, true, relGoal.matrix());
	pm(relGoal);

	// Perform IK
	bool result = ik(relGoal, phi, node.right);
	node.phi = phi;
	pv(node.right);
	return result;	
}

/* ********************************************************************************************** *
// The following are the start and goal configurations for the left arm: (1-2) line, (3) circle 
// (no rotation), (4) circle (rotation) when the robot is on the pedestal
double start_goals [][7] = {
	{-0.241886,  -1.90735,  -0.64163 ,  1.92837,         0,   -1.5166, -0.970454},
	{-0.238262,  -1.90277, -0.640441,   1.85266,         0,  -1.44662, -0.970454}, 
	{-0.21082, -1.86743,-0.631959,  2.26919     ,   0, -1.90708,-0.970454},
	{0.106257 ,   -1.4171,  -0.608521,   0.121866,-3.0488e-05,  -0.308467,  -0.970435}, 
  {0.0429373,  -1.40463, -0.254133,    1.5358,         0,  -1.70737,  -1.32024}, 
  {-0.241878,  -1.90734, -0.641629,   2.03699,         0,  -1.62524, -0.970454}, 
  {0.111058, -1.28436,  2.76566,  2.16177,        0, -1.85511,   1.9307}, 
	{-0.662844, -1.77665,  1.82688, 0.946289,        0, -1.63573,  2.81445},
};

/* ********************************************************************************************** */
// The following are the start and goal configurations for the left arm: (1) stick, (2) circle 
// when the robot is on the ground with imu = -1.87 and waist = 150. 
double start_goals [][7] = {
	{ 0.525674,    -1.57797,    -1.58317,     1.65143, -0.00998171,    -1.41622,     -1.5589},
	{ 1.15765 ,   -1.58036 ,   -1.57499 ,   0.959028, -0.00736767 ,   -1.35574 ,   -1.56166},
	{1.77443, -1.76594,  1.61081,  1.29396, -2.99145, -1.27951,  1.84021}, 
	{1.53664,  -1.7507,  1.56468, 0.886069, -3.01066, -1.45379,  1.50574},
	{0.490885,  -1.28308,  -2.87051, 2.03524,   -0.186148,  -1.68552,   1.11042}, // lever start: 80
	{0.410602, -1.7016, 3.07435, 1.54679, -0.343525, -1.69399, 0.463153}  // lever goal: 25
//	{0.507678, -1.38226, -2.93301, 1.95762, -0.21144, -1.7176, 0.980887} // lever goal: 70
 };

/* ********************************************************************************************** */
int main (int argc, char* argv[]) {

	srand(time(NULL));

	// Prepare the joint indices in forward kinematics function 
	for(size_t i = 10; i < 23; i+=2) left_idx.push_back(i);
	imuWaist_ids.push_back(5);	
	imuWaist_ids.push_back(8);	

	// Load the world
	DartLoader loader;
	world = loader.parseWorld("../../common/scenes/01-World-Robot.urdf");
	
	// Setup the start and goal nodes
	const size_t case_id = 0;
	Node start, goal;
	for(size_t i = 0; i < 7; i++) {
		start.left(i) = start_goals[2*case_id][i];
		goal.left(i) = start_goals[2*case_id+1][i];
	}
	
	// Perform I.K. for the start/goal nodes
	assert((computeIK(start, 0.0)) && "Could not compute I.K. for the start node");
	assert((computeIK(goal, 0.0)) && "Could not compute I.K. for the goal node");

	// Create the fr-rrt planner
	planner = new fr (world, krang_id, start, goal, boxLift_err_ground, lever_constraint);
	
	// Make the call
	list <Node*> path;
	bool result = planner->plan(path);
	planner->printPath(path);
	printf("%s with %lu nodes\n", result ? "good" : "not yet", path.size());	

	// Print the path
	Vector6d error;
	Matrix4d temp;
	for (list<Node*>::iterator it=path.begin(); it != path.end(); ++it) {
//		cout << planner->leftFrame.topRightCorner<3,1>().transpose() << endl;
		// pv(error);
	}
}
/* ********************************************************************************************** */
