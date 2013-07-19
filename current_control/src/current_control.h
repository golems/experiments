/**
 * @file current_control.h
 * @author Saul Reynolds-Haertle
 *         Stewart Butler
 * @date 2013-07-11

 * @brief header for some useful code for controlling the arms with
 * current directly instead of by going through the onboard
 * controllers.
 */

#pragma once

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <dynamics/SkeletonDynamics.h>

/* the size of the window used for pseudo-integral terms */
#define PID_ERROR_WINDOW_SIZE 30

/* a data type for holding on to the state of a single motor's PID
 * controller */
typedef struct {
    bool use_pos;
    bool use_vel;

    double pos_target;
    double vel_target;

    double output;

    double K_p_p;
    double K_p_d;
    double K_v_p;
    double K_v_d;
    double pos_error_last;
    double vel_error_last;
    double pos_error_window[PID_ERROR_WINDOW_SIZE];
    double vel_error_window[PID_ERROR_WINDOW_SIZE];
} pid_state_t;


/* function declarations */
void update_pids(somatic_motor_t* mot, pid_state_t* pids, double* result);
void do_init_pids(somatic_motor_t* mot, pid_state_t* pids);
void gravity_compensate(dynamics::SkeletonDynamics* robot, int* arm_node_ids, double* result);


// Local Variables:
// mode: c++
// End:
