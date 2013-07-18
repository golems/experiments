/**
 * @file current_control.cpp
 * @author Saul Reynolds-Haertle
 *         Stewart Butler
 * @date 2013-07-11

 * @brief useful code for controlling the arms with current directly
 * instead of by going through the onboard controllers.
 */

#include "current_control.h"

void do_init_pids(somatic_motor_t* mot, pid_state_t* pids) {
    for(int i = 0; i < mot->n; i++) {
        pids[i].pos_target = 0.0;
        pids[i].vel_target = 0.0;
        pids[i].pos_error_last = 0.0;
        pids[i].vel_error_last = 0.0;
        for(int j = 0; j < PID_ERROR_WINDOW_SIZE; j++) pids[i].pos_error_window[j] = 0.0;
        for(int j = 0; j < PID_ERROR_WINDOW_SIZE; j++) pids[i].vel_error_window[j] = 0.0;
        pids[i].pos_target = mot->pos[i];
    }
}

void update_pids(somatic_motor_t* mot, pid_state_t* pids, Eigen::VectorXd &result) {
    double p_p_value;
    double p_d_value;
    double v_p_value;
    double v_d_value;

    double pos_error;
    double vel_error;

    for(int i = 0; i < mot->n; i++) {
        result[i] = 0;

        if(pids[i].use_pos) {
            pos_error = pids[i].pos_target - mot->pos[i];

            p_p_value = pids[i].K_p_p * pos_error;
            p_d_value = pids[i].K_p_d * (pos_error - pids[i].pos_error_last);

            result[i] += p_p_value + p_d_value;

            pids[i].pos_error_last = pos_error;
        }
        if (pids[i].use_vel) {
            vel_error = pids[i].vel_target - mot->vel[i];

            v_p_value = pids[i].K_v_p * vel_error;
            v_d_value = pids[i].K_v_d * (vel_error - pids[i].vel_error_last);

            result[i] += v_p_value + v_d_value;

            pids[i].vel_error_last = vel_error;
        }
    }
}

void gravity_compensate(dynamics::SkeletonDynamics* robot, int* arm_node_ids, double* result) {
}
