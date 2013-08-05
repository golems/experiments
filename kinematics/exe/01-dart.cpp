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
	simulation::World* world = dl.parseWorld("../../common/scenes/00-World-Test.urdf");
	assert((world != NULL) && "Could not find the world");
	std::cout << "com: " << world->getSkeleton(0)->getWorldCOM().transpose() << std::endl;

	return 0 ;

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
	// Note that because wheels in dart setup cannot affect the end-effector directly we have
	// 24 - 7 (right arm) - 2 (wheels) = 15 dof in the jacobians.
	Eigen::MatrixXd Jlin = eeNode->getJacobianLinear();
	Eigen::MatrixXd Jang = eeNode->getJacobianAngular();
	printf("\nJlin size: (%d, %d)\n", Jlin.rows(), Jlin.cols());
	printf("Jang size: (%d, %d)\n", Jang.rows(), Jang.cols());
}
