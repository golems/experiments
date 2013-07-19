/**
 * @file rrt.cpp
 * @author Can Erdogan
 * @date May 22, 2013
 * @brief The RRT implementation for the upper body of Krang which involves only the two arms, also
 * ignoring the torso module. We assume that the two hands are holding an object. The idea is that
 * we can perform a 8-dof RRT where we first sample the 7-dof of the left arm and then we perform
 * an I.K. for the second arm where the 8th dof, the phi value used to parameterize teh redundancy
 * is alos sampled. For now, we only sample within a small window around the phi of the last node.
 * However, we should consider all the (16) configurations of I.K. in a smart way.
 */

#include <timing/timing.h>
#include "fr.h"
#include <fstream>

#define MAX_NUM_NODES ((int) (3e3)) // ((1e5))
#define MAX_NUM_PHIS 128 
#define PHI_RANGE (30.0 * M_PI / 180.0)

using namespace std;

/* ********************************************************************************************** */
bool fr::plan (std::list <Node*>& path) {

	// Set the options
	const double bias = 0.1;
	const double goalRadiusSq = SQ(0.05);//SQ(0.005);

	// Sanity check start and goal configurations with the error bounds
	epsilonSq = SQ(0.5);//SQ(0.05)
	Vector6d errorStart, errorGoal;
	task_error(start.left, errorStart);
	task_error(goal.left, errorGoal);
	assert((errorStart.squaredNorm() < epsilonSq) && "Start does not satisfy constraint!");
	assert((errorGoal.squaredNorm() < epsilonSq) && "Goal does not satisfy constraint!");

	tic_(everything);

	// Expand the tree until the limit of number of nodes is reached
	size_t numNodes = 1;
	Node target;
	double minDistanceSq = 1000000000000;
	while(numNodes < MAX_NUM_NODES - 1) {

		// cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv" << endl;
		if(numNodes % 10 == 0) cout << numNodes << " " << sqrt(minDistanceSq) << endl;

		// Choose a target: either random or the goal
		double biasRand = (((double) rand()) / RAND_MAX);
		if(biasRand < bias) target = goal;
		else {
			Vector7d random;
			randomConfig(random);
			target.left = random;
		}

		// Get the nearest neighbor
		tic(nearest);
		int nearest_id = nearestNeighbor(target.left);
		toc(nearest);

		// Create a new sample for the left arm. If not possible, repeat.
		tic(constrainedLeft);
		Node newNode;
		bool result = constrainedLeft(nodes[nearest_id].left, target.left, newNode.left);
		toc(constrainedLeft);
		if(!result) continue;

		// Perform I.K. for the right arm and get the phi value. If not possible, repeat.
		if(twoHands && !ikRight(nodes[nearest_id], newNode)) 
			continue;

		// Add to the tree
		nodes[numNodes] = newNode;
		parent_ids[numNodes] = nearest_id;
		tic(addPoints);
		tree->addPoints(flann::Matrix<double>((double*)nodes[numNodes].left.data(), 1, 7));
		toc(addPoints);

		// Check for the end condition
		double goalDistanceSq = (goal.left - newNode.left).squaredNorm();
		minDistanceSq = min(goalDistanceSq, minDistanceSq);
		if(goalDistanceSq < goalRadiusSq) {
			printf("Done with %lu nodes and goal distance %lf\n", numNodes, sqrt(goalDistanceSq));
			toc_(everything);
			tictoc_print_();
			tracePath(numNodes, path);
			return true;
		}

		tictoc_finishedIteration_();
		numNodes++;
	}

	toc_(everything);
	tictoc_print_();

	return false;
}

/* ********************************************************************************************** */
bool fr::ikRight (const Node& nearestNode, Node& newNode) {

	static const bool debug = 0;

	// Get where the right hand should be given the left hand's frame
	if(debug) pv(newNode.left);
	if(debug) pv(matToVec(leftFrame));
	static Matrix4d rightFrame;
	twoHandConstraint(leftFrame, rightFrame);
	if(debug) pv(matToVec(rightFrame));	
	// Get the realtive goal
	static Transform <double, 3, Affine> relGoal;
	getWristInShoulder(robot, rightFrame, true, relGoal.matrix());

	// Sample phi values with uniform probability from the window
	Matrix <double, 7, 1> theta;	
	for(size_t phi_idx = 0; phi_idx < MAX_NUM_PHIS; phi_idx++) {

		// Select a phi
		double phi = nearestNode.phi + PHI_RANGE * ((((double) rand()) / RAND_MAX) - 0.5);
		if(debug) cout << "trying phi: " << phi << endl;

		// Perform inverse kinematics
		bool result = ik(relGoal, phi, theta);
		if(result) {

			// Check collision
			vector <int> arm_ids;
			for(size_t i = 4; i < 17; i+=2) arm_ids.push_back(i + 6 + 1);  
			robot->setConfig(arm_ids, theta);
			bool collision = world->checkCollision(false);
			if(collision) continue; 

			// Check if the right arm configuration is too far from nearest
			Vector7d diff = theta - nearestNode.right;
			double maxDiff = diff.lpNorm<Infinity>();
			if(maxDiff > 0.5) continue; 

			// Add the node if no problems
			newNode.phi = phi;
			newNode.right = theta;
			return true;
		}
	}
	
	if(debug) cout << "sad" << endl;
	// exit(0);
	return false;
}

/* ********************************************************************************************** */
/// Create a new sample for first-order retraction
inline void fr::retractConfig(Vector7d& sample, const Vector6d& error) {

	// Get the Jacobian
	kinematics::BodyNode* eeNode = world->getSkeleton(0)->getNode("lgPlate1");
	MatrixXd Jlin = eeNode->getJacobianLinear().topRightCorner<3,7>();
	MatrixXd Jang = eeNode->getJacobianAngular().topRightCorner<3,7>();
	MatrixXd J (6,7);
	J << Jlin, Jang;

	// Change the sample
	MatrixXd Jinv = J.transpose() * (J*J.transpose()).inverse();
	Vector7d dq = Jinv * error;
	sample += dq;
}

/* ********************************************************************************************** */
inline bool fr::repeatedRetraction(Vector7d& sample, const Vector7d& nearest) {

	// Compute the task error with the current sample and save it as the initial sample
  Vector6d error;
	task_error(sample, error);
	Vector7d& initSample = sample;
	double initDistanceSq = (initSample - nearest).squaredNorm();

	// Keep creating new samples until current error has decreased enough
	const size_t maxNumSamples = 100;
	for(size_t i = 0; i < maxNumSamples; i++) {

		// Check if the error is small enough
		if(error.squaredNorm() <= epsilonSq) return true;

		// Get a new sample 
		retractConfig(sample, error);

		// Update the current error
		task_error(sample, error);

		// Check if the retracted sample is further away from the nearest sample compared to initial
		if((sample - nearest).squaredNorm() > initDistanceSq) return false;
	}

	// Return false if a good candidate is not found in the max iterations
	return false;
}

/* ********************************************************************************************** */
inline bool fr::constrainedLeft (const Vector7d& nearest, const Vector7d& target, Vector7d& left) {

	// Get the direction and distance between nearest and target
	Vector7d dir = target - nearest;
	double distance = dir.norm();	
	dir /= distance;

	// First create a sample by extending towards the target, ignoring the constraint	
	const double deltaT = 0.1; // 0.001
	if(distance < deltaT) left = target;
	else left = nearest + deltaT * dir;

	// Then project the sample to the constrain manifold if possible
	bool constrained = repeatedRetraction(left, nearest);
	if(!constrained) return false;

	// Finally, check if the projected sample is within the joint limits
	if((left.minCoeff() < -M_PI) || (left.maxCoeff() > M_PI)) return false;
	return true;
}

/* ********************************************************************************************** */
inline int fr::nearestNeighbor(const Vector7d& sample) {
	static int nearest_id;
	static flann::Matrix<int> nearestMatrix(&nearest_id, 1, 1);
	static double temp;
	static flann::Matrix<double> distanceMatrix(flann::Matrix<double>(&temp, 1, 1));
	static flann::SearchParams sParams (32, 0., true);
	const flann::Matrix<double> queryMatrix((double*)sample.data(), 1, 7);
	tree->knnSearch(queryMatrix, nearestMatrix, distanceMatrix, 1, sParams);
	return nearest_id;
}

/* ********************************************************************************************** */
inline void fr::randomConfig (Vector7d& random) {
	random = M_PI * Vector7d::Random();
}

/* ********************************************************************************************** */
void fr::tracePath(int node, list<Node*>& path, bool reverse) {

	// Keep following the "linked list" in the given direction
	int x = node;
	while(x != -1) {
		if(!reverse) path.push_front(&nodes[x]);	
		else path.push_back(&nodes[x]);
		x = parent_ids[x];
	}
}

/* ********************************************************************************************** */
void fr::printPath(const list<Node*>& path) {
	cout << "two: " << twoHands << endl;
	ofstream ofs ("data", ofstream::out);
	VectorXd temp = VectorXd::Zero(24);
	temp(2) = 0.30;
	temp(3) = -1.5708;
	temp(5) = 1.857-3.0 * M_PI_2; //1.5708;
	temp(8) = (5.0 / 6) * M_PI;
	for(list<Node*>::const_iterator it=path.begin(); it != path.end(); ++it) {
		for(size_t i = 10, j = 0; i < 23; i+=2, j++) temp(i) = (*it)->left(j);
		if(twoHands)
			for(size_t i = 10, j = 0; i < 23; i+=2, j++) 
				temp(i+1) = (*it)->right(j);
		cout << (*it)->right.transpose() << endl;
		ofs << temp.transpose();
		ofs << "\n";
	}
	ofs.close();
}

/* ********************************************************************************************** */
fr::fr (simulation::World* _world, size_t robot_idx, const Node& _start, const Node& _goal,
		err_func _task_error, constraint_func _twoHandConstraint) {

	// Set the world, the robot, the goal and start configurations, and the task error function
	world = _world;
	robot = world->getSkeleton(robot_idx);
	task_error = _task_error;
	start = _start;
	goal = _goal;
	twoHandConstraint = _twoHandConstraint;
	twoHands = (twoHandConstraint != NULL);
	
	// Setup the node and parent vectors
	parent_ids = new int [MAX_NUM_NODES];
	nodes = new Node [MAX_NUM_NODES];

	// Setup the flann structure
	tree = new flann::Index <flann::L2<double> > (flann::KDTreeIndexParams());

	// Populate the tree with the start node
	nodes[0].left = start.left;
	nodes[0].right = start.right;
	nodes[0].phi = start.phi;
	parent_ids[0] = -1;
	tree->buildIndex(flann::Matrix<double>((double*)nodes[0].left.data(), 1, 7));
} 
