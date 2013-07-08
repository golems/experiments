/**
 * @file helpers.h
 * @author Munzir
 * @date July 8th, 2013
 * @brief This file comtains some helper functions used for balancing	
 */

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>

#include <amino.h>
#include <ach.h>
#include <dirent.h>
#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>

#include <filter.h>
#include <imud.h>
#include <pciod.h>

#include "Dynamics.h"

/*************************************************************************************************/
/// Filters the imu, wheel and waist readings
void filterState(double dt, filter_kalman_t *kf, Eigen::VectorXd& q, Eigen::VectorXd& dq);

/// Reads imu values from the ach channels and computes the imu values
void getImu(double& _imu, double& _imuSpeed); 
