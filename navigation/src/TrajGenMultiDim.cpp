#include "TrajGenMultiDim.h"

#define ACC_RATIO 0.10
#define DEC_RATIO 0.20

#define MAX(x, y) ((x) >= (y) ? (x) : (y))

#include <iostream>

using namespace std;

/*
 * Returns the angle expressed in radians between -Pi and Pi
 */ 
double unwrap(double angle) 
{   
    int osign = angle >= 0 ? 1 : -1;
    return osign * (fmod(abs(angle) + M_PI, 2 * M_PI) - M_PI);
}

TrajGenMultiDim::TrajGenMultiDim() {
    //_T_a = 0;
    //_T_d = 0;
    _T = 0;
    //_sDot_max = 0;

    _vel_avg_lin = 0.25;  // m/sec
    _vel_avg_rot = 0.35; // rad/sec

    return;
}

void TrajGenMultiDim::setTargetState(const Vector6d& start_state, const Vector6d& target_state) {

    // Step 1: Calculate rotational time and linear time. Choose time whichever
    //  is larger.
    double dist_lin = (start_state.head(2) - target_state.head(2)).norm();
    double dist_rot = abs(unwrap(target_state[2] - start_state[2]));

    _T = MAX(dist_lin/_vel_avg_lin, dist_rot/_vel_avg_rot);
    
    _trajGen_x.initializeTrajectory(start_state[3], target_state[0] - start_state[0], _T);
    _trajGen_y.initializeTrajectory(start_state[4], target_state[1] - start_state[1], _T);
    _trajGen_theta.initializeTrajectory(start_state[5], unwrap(target_state[2] - start_state[2]), _T);

    //_trajGen_x.initializeTrajectory(0, target_state[0] - start_state[0], _T);
    //_trajGen_y.initializeTrajectory(0, target_state[1] - start_state[1], _T);
    //_trajGen_theta.initializeTrajectory(0, unwrap(target_state[2] - start_state[2]), _T);


    // Step 2: Set the parameters of motion profile.
    //_T_a = ACC_RATIO * _T;
    //_T_d = DEC_RATIO * _T;
//
//    _sDot_max = 2/(2*_T - _T_a - _T_d);
//
    _start_state = start_state;
    //_end_state = target_state;
    
    return;
}

Vector6d TrajGenMultiDim::getReferenceState(double t) {

    Vector6d refState;
//    double s = 0, sDot = 0;
//
//    if (t < 0) { //return starting pose
//        s = 0;
//        sDot = 0;
//    }
//    else if (t < _T_a) {
//        s = _sDot_max * t * t / (2*_T_a);
//        sDot = _sDot_max * t / _T_a;
//    }
//    else if (t < (_T - _T_d)) {
//        s = _sDot_max * (t - _T_a/2);
//        sDot = _sDot_max;
//    }
//    else if (t < _T) {
//        s = _sDot_max * ( t - _T_a/2 - (t - _T + _T_d) * (t - _T + _T_d) / (2*_T_d) );
//        sDot = - _sDot_max * (t - _T)/_T_d;
//    }    
//    else {
//        s = 1;
//        sDot = 0;
//    } 

//    refState[0] = _start_state[0] + (_end_state[0] - _start_state[0]) * s;
//    refState[1] = _start_state[1] + (_end_state[1] - _start_state[1]) * s;
//    refState[2] = unwrap(_start_state[2] + unwrap(_end_state[2] - _start_state[2]) * s);
//    refState[3] = (_end_state[0] - _start_state[0]) * sDot;
//    refState[4] = (_end_state[1] - _start_state[1]) * sDot;
//    refState[5] = unwrap(_end_state[2] - _start_state[2]) * sDot;

    float x_disp, x_vel;
    float y_disp, y_vel;
    float theta_disp, theta_vel;

    // Get the displacements and velocities
    _trajGen_x.getReferenceState(t, x_disp, x_vel);
    _trajGen_y.getReferenceState(t, y_disp, y_vel);
    _trajGen_theta.getReferenceState(t, theta_disp, theta_vel);

    refState[0] = _start_state[0] + x_disp;
    refState[1] = _start_state[1] + y_disp;
    refState[2] = unwrap(_start_state[2] + theta_disp);\
    refState[3] = x_vel;    
    refState[4] = y_vel;
    refState[5] = theta_vel;    

    return refState;
}

double TrajGenMultiDim::getTotalTime() {
    return _T;
}
