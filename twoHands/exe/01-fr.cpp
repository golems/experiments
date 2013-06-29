/**
 * @file 05-fr.cpp
 * @author Can Erdogan
 * @date May 19, 2013
 * @brief This executable demonstrates the first-order retraction method for task constrained
 * motion planning for Krang, trying to open a valve.
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

vector <int> left_idx;
World* world;

fr* planner;

/* ********************************************************************************************** */
/// Performs forward kinematics and returns the position and orientation (quaternion) of the
/// left end-effector
void forward (const Vector7d& node, Vector3d& eePos, Quaternion <double>& eeOri) {
	world->getSkeleton(1)->setConfig(left_idx, node);
	kinematics::BodyNode* eeNode = world->getSkeleton(1)->getNode("lgPlate1");
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
	Matrix3d constOriM = math::eulerToMatrix(constRPY, math::XYZ);
	Quaternion <double> constOri (constOriM); 
 
	// Find the orientation error and express it in RPY representation
	Quaternion <double> errOriQ = constOri * eeOri.inverse();
	Matrix3d errOriM = errOriM = errOriQ.matrix();
	Vector3d errOri = math::matrixToEuler(errOriM, math::XYZ);

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
/// Given that the left hand is holding a stick perpendicular to its x axis and extending to its
/// y axis, and the right hand would hold it again perpendicular to its x axis but extending
/// to its -y axis, returns where the right hand should be.
void stick_constraint (const Matrix4d& left, Matrix4d& right) {
	right = left;
	right.topRightCorner<4,1>() -= left.block<4,1>(0,0).normalized() * 0.3;
}

/* ********************************************************************************************** */
/// Computes I.K. for a given node and phi value
bool computeIK (Node& node, double phi) {

	// Perform forward kinematics and determine where the right frame should be
	world->getSkeleton(1)->setConfig(left_idx, node.left);
	Matrix4d leftFrame = world->getSkeleton(1)->getNode("lgPlate1")->getWorldTransform(), rightFrame;
	// pmr(leftFrame);
	stick_constraint(leftFrame, rightFrame);
	// pmr(rightFrame);

	pv(node.left);

	// Get the relative goal
	static Transform <double, 3, Affine> relGoal;
	getWristInShoulder(world->getSkeleton(1), rightFrame, true, relGoal.matrix());

	// Perform IK
	bool result = ik(relGoal, phi, node.right);
	node.phi = phi;
	pv(node.right);
	return result;	
}

/* ********************************************************************************************** */
// The following are the start and goal configurations for the left arm: (1-2) line, (3) circle 
// (no rotation), (4) circle (rotation)
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
int main (int argc, char* argv[]) {

	srand(time(NULL));

	// Prepare the line error
	for(size_t i = 10; i < 23; i+=2) left_idx.push_back(i);

	// Load the world
	DartLoader loader;
	world = loader.parseWorld("/home/cerdogan/Documents/MacGyver/3rdParty/simulation/scenes/"
		"07-World-FR.urdf");
	
	// Setup the start and goal nodes
	const size_t case_id = 3;
	Node start, goal;
	for(size_t i = 0; i < 7; i++) {
		start.left(i) = start_goals[2*case_id][i];
		goal.left(i) = start_goals[2*case_id+1][i];
	}
	
	// Perform I.K. for the start/goal nodes
	assert((computeIK(start, 0.0)) && "Could not compute I.K. for the start node");
	assert((computeIK(goal, 0.0)) && "Could not compute I.K. for the goal node");

	// Create the fr-rrt planner
	planner = new fr (world, 1, start, goal, plane_err, stick_constraint);
	
	// Make the call
	list <Node*> path;
	bool result = planner->plan(path);
	planner->printPath(path);
	printf("%s with %lu nodes\n", result ? "good" : "not yet", path.size());	

	// Print the path
	Vector6d error;
	Matrix4d temp;
	for (list<Node*>::iterator it=path.begin(); it != path.end(); ++it) {
		plane_err((*it)->left, error);
//		cout << planner->leftFrame.topRightCorner<3,1>().transpose() << endl;
		// pv(error);
	}
}
/* ********************************************************************************************** */
