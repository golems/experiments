#include "TrajGen.h"

#define ACC_RATIO 0.2

#define MAX(x, y) ((x) >= (y) ? (x) : (y))

TrajGen::TrajGen() {
    _T_a = 0;
    _T_d = 0;
    _T = 0;
    _sDot_max = 0;

    _vel_avg_lin = 0.2;
    _vel_avg_rot = 0.35;
    return;
}

void TrajGen::setTargetState(const Vector6d& start_state, const Vector6d& target_state) {

    // Step 1: Calculate rotational time and linear time. Choose time whichever
    //  is larger.
    double dist_lin = (start_state.head(2) - target_state.head(2)).norm();
    double dist_rot = abs(target_state[2] - start_state[2]);

    _T = MAX(dist_lin/_vel_avg_lin, dist_rot/_vel_avg_rot);

    // Step 2: Set the parameters of motion profile.
    _T_a = ACC_RATIO * _T;
    _T_d = ACC_RATIO * _T;

    _sDot_max = 2/(2*_T - _T_a - _T_d);

    _start_state = start_state;
    _end_state = target_state;
    
    return;
}

Vector6d TrajGen::getReferenceState(double t) {

    Vector6d refState;
    double s = 0, sDot = 0;

    if (t < 0) { //return starting pose
        s = 0;
        sDot = 0;
    }
    else if (t < _T_a) {
        s = _sDot_max * t * t / (2*_T_a);
        sDot = _sDot_max * t / _T_a;
    }
    else if (t < (_T - _T_d)) {
        s = _sDot_max * (t - _T_a/2);
        sDot = _sDot_max;
    }
    else if (t < _T) {
        s = _sDot_max * ( t - _T_a/2 - (t - _T + _T_d) * (t - _T + _T_d) / (2*_T_d) );
        sDot = - _sDot_max * (t - _T)/_T_d;
    }    
    else {
        s = 1;
        sDot = 0;
    } 

    refState[0] = _start_state[0] + (_end_state[0] - _start_state[0]) * s;
    refState[1] = _start_state[1] + (_end_state[1] - _start_state[1]) * s;
    refState[2] = _start_state[2] + (_end_state[2] - _start_state[2]) * s;
    refState[3] = (_end_state[0] - _start_state[0]) * sDot;
    refState[4] = (_end_state[1] - _start_state[1]) * sDot;
    refState[5] = (_end_state[2] - _start_state[2]) * sDot;

    return refState;
}