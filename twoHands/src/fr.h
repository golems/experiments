/**
 * @file rrt.h
 * @author Can Erdogan
 * @date May 22, 2013
 * @brief The FR-RRT implementation for the upper body of Krang which involves only the two arms, 
 * ignoring the torso module. We assume that the two hands are holding an object. The idea is that
 * we can perform a 8-dof RRT where we first sample the 7-dof of the left arm and then we perform
 * an I.K. for the second arm where the 8th dof, the phi value used to parameterize teh redundancy
 * is alos sampled. For now, we only sample within a small window around the phi of the last node.
 * However, we should consider all the (16) configurations of I.K. in a smart way.
 * todos for speed up:
 * 0- bidirectional
 * 1- approximate nearest neighbor
 * 2- do not create data structures unnecessarily (send static vectors as reference parameters)
 * 3- don't check for goal every iteration
 * 4- parallelize
 * 5- check if you can increase MAX_NUM_NODES in the stack
 * 6- reuse the distance computed by flann between the nearest and random
 * 7- redirect the samples from the reachability of both arms
 */

#include <list>

#include <Eigen/Dense>
#include <flann/flann.hpp>

#include <kinematics/BodyNode.h>
#include <simulation/World.h>

#include "ik.h"

using namespace Eigen;
using namespace simulation;
using namespace dynamics;

typedef Matrix <double, 7, 1> Vector7d;
typedef Matrix <double, 6, 1> Vector6d;
typedef void (*err_func) (const Vector7d&, Vector6d&);
typedef void (*constraint_func) (const Matrix4d&, Matrix4d&);

#define DEG2RAD(x) ((x) / 180.0 * M_PI)
#define SQ(x) ((x) * (x))
#define pc(a) std::cout << #a << ": " << (a) << "\n" << std::endl

/// The index of the robot skeleton in world array
static const int r_id = 0;

/// The configuration of a node
struct Node {
	/// To get byte-aligned Eigen vectors
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Vector7d left, right;
	double phi;
};

/// The rapidly-expanding random tree implementation
class fr {
public:
	/// To get byte-aligned Eigen vectors
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// The inputs
	Node start;																///< The start node
	Node goal;																///< The goal node
	simulation::World* world;            		  ///< The world that the robot is in
	dynamics::SkeletonDynamics* robot;   		  ///< The ID of the robot for which a plan is generated
	err_func task_error;											///< The task constraint function for the left hand
	constraint_func twoHandConstraint;				///< The constraint between the two hands

	// Planner variables
	int* parent_ids;														///< The parent indices
	Node* nodes;																///< All visited configs
	flann::Index <flann::L2 <double> >* tree;		///< For fast nearest neighbor searches 

	// Constants
	double epsilonSq;
	Transform <double, 3, Affine> A, B;					///< The transformation matrices for I.K.
	Matrix4d leftFrame;				///< The f.k. of the left in the last good config set by error function
	bool twoHands;						///< Indicates if we should perform I.K. for the right hand

public:

	/// Create a constrained sample towards the target 
	bool constrainedLeft (const Vector7d& nearest, const Vector7d& target, Vector7d& left);
	
	/// Given a constraint function, a start and goal configuration, plans a route.
	bool plan (std::list <Node*>& path);

	/// The constructor
	fr (simulation::World* world, size_t robot_idx, const Node& start, const Node& goal,
			err_func task_error, constraint_func twoHandConstraint = NULL);

	/// Returns the nearest neighbor to query point
	int nearestNeighbor(const Vector7d& sample);

	/// Returns a random config for the left arm
	void randomConfig (Vector7d& random);

	/// Attempts to project a sample to the constraint manifold 
	void retractConfig(Vector7d& sample, const Vector6d& error);

	/// Attempts to repeatedly project a sample to the constraint manifold. If the projected sample,
	/// actually moves away too far from the nearest sample, decides failure. (why?)
	bool repeatedRetraction(Vector7d& sample, const Vector7d& nearest);

	/// Given a candidate node with a selected 7-dof, perform I.K. with a phi value in the window of 
	/// the nearest neighbors and check for collisions. If I.K. fails with MAX_NUM_PHIS due to reach
	/// or collision detections, return false so that a new random left arm configuration is sampled.
	bool ikRight (const Node& nearestNode, Node& newNode);

	/// Traces the path from some node to the initConfig node - useful in creating the full path
	/// after the goal is reached.
	void tracePath(int node, std::list<Node*>& path, bool reverse = false);

	/// Print the path to a file 
	void printPath(const std::list<Node*>& path);
};
