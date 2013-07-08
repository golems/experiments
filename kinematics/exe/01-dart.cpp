/**
 * @file forwardKinematics.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date July 08, 2013
 * @brief This executable shows how to use dart structure to represent a robot and get the 
 * jacobian. Note that it uses the world in experiments.git/common/scenes/01-World.Robot.urdf.
 */

#include <kinematics/BodyNode.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <iostream>
#include <vector>

#include <Eigen/Dense>

int main () {

	// Create the world which has the robot definition
	DartLoader dl;
	simulation::World* world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");

	// Get the robot pointer
	dynamics::SkeletonDynamics* robot = world->getSkeleton(0);

	// Set the dofs of the robot
	std::vector <int> all_dofs;
	for(int i = 0; i < 24; i++) all_dofs.push_back(i);
	Eigen::VectorXd set_all_conf (24);
	for(int i = 0; i < 24; i++) set_all_conf(i) = M_PI / 180 * i;
	robot->setConfig(all_dofs, set_all_conf);

	// Print the dofs of the robot
	Eigen::VectorXd all_conf = robot->getConfig(all_dofs);
	std::cout << "Read values: " << all_conf.transpose() << std::endl;
	std::cout << "Set values: " << set_all_conf.transpose() << std::endl;
	
	// Get the left end-effector node - a robot is made out of nodes and connecting joints
	kinematics::BodyNode* eeNode = robot->getNode("lGripper");

	// Get the linear and angular jacobian of the left end-effector and print their sizes
	Eigen::MatrixXd Jlin = eeNode->getJacobianLinear();
	Eigen::MatrixXd Jang = eeNode->getJacobianAngular();
	printf("\nJlin size: (%d, %d)\n", Jlin.rows(), Jlin.cols());
	printf("Jang size: (%d, %d)\n", Jang.rows(), Jang.cols());
}
