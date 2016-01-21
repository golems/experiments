#include "TrajGen.h"

#define ACC_RATIO 0.10
#define DEC_RATIO 0.20

#define MAX(x, y) ((x) >= (y) ? (x) : (y))

#include <iostream>

using namespace std;

#define IS_NEAR_ZERO(x, precision) (abs(x) < (precision))

TrajGen::TrajGen() {
    _T_a = 0;
    _T_d = 0;
    _T = 0;
    _v_i = 0;
    _v_cruise = 0;

    return;
}

//void TrajGenMultiDim::setTargetState(const Vector6d& start_state, const Vector6d& target_state) {

void TrajGen::initializeTrajectory(float v_i, float D, float T) {

    // Step 2: Set the parameters of motion profile.
    _T = T;
    _T_a = ACC_RATIO * T;
    _T_d = DEC_RATIO * T;

    _v_cruise = IS_NEAR_ZERO(_T_a, 0.0001) ? 0 : (2*D - v_i*_T_a)/(2*_T - _T_a - _T_d);

    //_sDot_max = 2/(2*_T - _T_a - _T_d);

    _v_i = v_i;

    //printf("_T_a: %f _T_d: %f _T: %f _v_cruise: %f _v_i: %f\n", 
    //       _T_a, _T_d, _T, _v_cruise, _v_i);

    return;
}


//Vector6d TrajGen::getReferenceState(double t) {
void TrajGen::getReferenceState(double t, float& x_ref, float& v_ref) {

    //Vector6d refState;
    //double s = 0, sDot = 0;
    if (t < 0) { //return starting pose
        x_ref = 0;
        v_ref = 0;
    }
    else if (t < _T_a) {
        //s = _sDot_max * t * t / (2*_T_a);
        v_ref = _v_i + t * (_v_cruise - _v_i)/_T_a;
        x_ref = (_v_i + v_ref) * t/2;
        //sDot = _sDot_max * t / _T_a;
    }
    else if (t < (_T - _T_d)) {
        v_ref = _v_cruise;
        x_ref = (_v_i - _v_cruise) * _T_a/2 + t*_v_cruise;
        //s = _sDot_max * (t - _T_a/2);
        //sDot = _sDot_max;
    }
    else if (t < _T) {
        v_ref = (_T - t) * _v_cruise/_T_d;
        x_ref = (_v_i + _v_cruise) * _T_a/2
                + (_T - _T_a - _T_d) * _v_cruise
                + (_v_cruise + v_ref) * (t - _T + _T_d)/2;

        //s = _sDot_max * ( t - _T_a/2 - (t - _T + _T_d) * (t - _T + _T_d) / (2*_T_d) );
        //sDot = - _sDot_max * (t - _T)/_T_d;
    }    
    else {
        x_ref = (_v_i + _v_cruise) * _T_a/2 + _v_cruise * (_T - _T_a - _T_d) + _v_cruise * _T_d/2;
        v_ref = 0;
    } 

    //refState[0] = _start_state[0] + (_end_state[0] - _start_state[0]) * s;
    //refState[1] = _start_state[1] + (_end_state[1] - _start_state[1]) * s;
    //refState[2] = unwrap(_start_state[2] + unwrap(_end_state[2] - _start_state[2]) * s);
    //refState[3] = (_end_state[0] - _start_state[0]) * sDot;
    //refState[4] = (_end_state[1] - _start_state[1]) * sDot;
    //refState[5] = unwrap(_end_state[2] - _start_state[2]) * sDot;

    return;
}

double TrajGen::getTotalTime() {
    return _T;
}