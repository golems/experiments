/**
 * @file 04-force-compliance.cpp
 * @author Saul Reynolds-Haertle
 *         Stewart Butler
 * @date 2013-07-13

 * @briefs This executable demonstrates current-controlled force
 * compliance, like compliance/06-simplyComply but with current
 * controls.
 */

#include "current_control.cpp"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <imud.h>
#include <ssdmu/ssdmu.h>

#include <Eigen/Dense>

// DART crap
#include <kinematics/Joint.h>
#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

// ################################################################################
// ################################################################################
// DECLARATIONS
// ################################################################################
// ################################################################################

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 7, 1> Vector7d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

double ssdmu_pitch(double x, double y, double z);
double get_imu();
Eigen::Vector6d get_ft();

void update_motor_state();
void update_dart_state();

Eigen::Vector6d compute_external_force(const Eigen::Vector6d& input);
Eigen::Vector6d compute_ft_offset(const Eigen::Vector6d& raw_ft_reading);

void run();
void init();
void destroy();

#define DISPLAY_VECTOR(VEC) std::cout << std::setw(25) << std::left << #VEC; for(int i = 0; i < VEC.size(); i++) std::cout << std::setw(12) << VEC[i]; std::cout << std::endl;

// ################################################################################
// ################################################################################
// GLOBALS
// ################################################################################
// ################################################################################

somatic_d_t daemon_cx;
ach_channel_t ft_chan;
ach_channel_t waist_chan; 
ach_channel_t imu_chan;

somatic_motor_t rlwa;
somatic_motor_t waist;
double imu_angle;
double waist_angle;

Eigen::Vector6d r_ft_offset;

simulation::World* world;
dynamics::SkeletonDynamics* krang;
kinematics::BodyNode* end_effector_node;

pid_state_t rpids[7];

// ################################################################################
// ################################################################################
// CONSTANTS
// ################################################################################
// ################################################################################

// whether we actually send commands
bool do_send_cmds = true;

// PID controller values
bool use_pos[] = {true, true, true, true, true, true, true};
bool use_vel[] = {true, true, true, true, true, true, true};

double init_K_p_p[] = {15.0,  15.0, 15.0, 12.0, 15.0,  7.0,  7.0};
double init_K_p_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
double init_K_v_p[] = {1.0,   1.0,  1.0,  1.0,  1.0,  1.0,  1.0};
double init_K_v_d[] = {0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0};

// indices of joints in DART
int rarm_ids_initializer[] = {12, 14, 16, 18, 20, 22, 24};
int waist_ids_initializer[] = {8};
int imu_ids_initializer[] = {5};
std::vector<int> rarm_ids;
std::vector<int> waist_ids;
std::vector<int> imu_ids;

// number of iterations to use to gather clean data at beginning of
// run
int ft_init_iters = 100;
int imu_init_iters = 500;

// position of schunk gripper's com in end effector frame of reference
// 0.065 robotiq itself, 0.026 length of ext + 2nd
static const Eigen::Vector3d schunk_gripper_com_vector(0.0, -0.008, 0.091);

// mass of the end effector
double end_effector_mass = 2.3 + 0.169 + 0.000;

// name of our end effector node
char* end_effector_node_name = "rGripper";

// how many times per second we update our FT readings
double ft_update_freq = 1000.0;

// thresholds for force-torqe readings. We ignore anythign less than
// these. Units are newtons and newton-meters.
double compliance_threshold_force = 4.0;
double compliance_threshold_torque = .4;

// how many times per second to print information to the screen
double display_freq = 10.0;

// Angle that the DMU is mounted at: 45 degrees.
double csr = -.7853981634;

// #############################################################################
// #############################################################################
// HELPERS
// #############################################################################
// #############################################################################

double timespec_to_double(struct timespec x) {
    return (double)x.tv_sec + (double)x.tv_nsec / 1000000000.0;
}

double gettime() {
    struct timespec temp;
    clock_gettime(ACH_DEFAULT_CLOCK, &temp);
    return timespec_to_double(temp);
}

double ssdmu_pitch(double x, double y, double z) {
    double newX;
    newX = x*cos(csr) - y*sin(csr);
    return atan2(newX, z); 
}

Eigen::Vector7d wrench_to_joint_vels(const Eigen::Vector6d& wrench) {
    // Get the Jacobian for computing joint-space velocities
    const char* nodeName = "rGripper";
    Eigen::MatrixXd jacobian_linear = end_effector_node->getJacobianLinear().topRightCorner<3,7>();
    Eigen::MatrixXd jacobian_angular = end_effector_node->getJacobianAngular().topRightCorner<3,7>();
    Eigen::MatrixXd jacobian (6,7);
    jacobian << jacobian_linear, jacobian_angular;

    // Compute the inverse of the Jacobian
    Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
    Eigen::MatrixXd jacobian_inverse = jacobian_transpose * (jacobian * jacobian_transpose).inverse();

    // Get the joint-space velocities by multiplying inverse Jacobian
    // with the opposite wrench.
    Eigen::Vector7d jointspace_velocities = jacobian_inverse * wrench / 300.0;

    // Threshold the velocities
    for(size_t i = 0; i < 7; i++) {
        if(jointspace_velocities(i) > 0.1) jointspace_velocities(i) = 0.2;
        else if(jointspace_velocities(i) < -0.1) jointspace_velocities(i) = -0.2;
    }

    return jointspace_velocities;
}

Eigen::Vector6d compute_ft_offset(const Eigen::Vector6d& raw_ft_reading) {
    // Get the point transform wrench due to moving the affected
    // position from com to sensor origin. The transform is an
    // identity with the bottom left a skew symmetric of the point
    // translation
    Eigen::Matrix6d transform_eecom_sensor = Eigen::MatrixXd::Identity(6,6); 
    transform_eecom_sensor.bottomLeftCorner<3,3>() <<
        0.0, -schunk_gripper_com_vector(2), schunk_gripper_com_vector(1),
        schunk_gripper_com_vector(2), 0.0, -schunk_gripper_com_vector(0), 
        -schunk_gripper_com_vector(1), schunk_gripper_com_vector(0), 0.0;

    // figure out how our end effector is rotated by giving dart the
    // arm values and the imu/waist values
    update_dart_state();
    Eigen::MatrixXd ee_world_transform = end_effector_node->getWorldTransform();
    Eigen::Matrix3d ee_rotation = ee_world_transform.topLeftCorner<3,3>().transpose();
	
    // Create the wrench with computed rotation to change the frame
    // from the world to the sensor.
    Eigen::Matrix6d wrenchrotate_sensor_world = Eigen::MatrixXd::Identity(6,6);
    wrenchrotate_sensor_world.topLeftCorner<3,3>() = ee_rotation;
    wrenchrotate_sensor_world.bottomRightCorner<3,3>() = ee_rotation;
	
    // Get the weight vector (note that we use the world frame for
    // gravity so towards -y)
    Eigen::Vector6d end_effector_weight_in_world;
    end_effector_weight_in_world << 0.0, end_effector_mass * -9.81, 0.0, 0.0, 0.0, 0.0;
	
    // Compute what the force and torque should be without any external values by multiplying the 
    // position and rotation transforms with the expected effect of the gravity 
    Eigen::Vector6d expected_ft_reading = transform_eecom_sensor * wrenchrotate_sensor_world * end_effector_weight_in_world;

    // DISPLAY_VECTOR(expected_ft_reading);

    // Compute the difference between the actual and expected f/t
    // values and return the result
    Eigen::Vector6d offset = expected_ft_reading - raw_ft_reading;
    return offset;
}

Eigen::Vector6d compute_external_force(const Eigen::Vector6d& input) {
    // Get the point transform wrench due to moving the affected
    // position from com to sensor origin. The transform is an
    // identity with the bottom left a skew symmetric of the point
    // translation
    Eigen::Matrix6d transform_eecom_sensor = Eigen::MatrixXd::Identity(6,6); 
    transform_eecom_sensor.bottomLeftCorner<3,3>() <<
        0.0, -schunk_gripper_com_vector(2), schunk_gripper_com_vector(1),
        schunk_gripper_com_vector(2), 0.0, -schunk_gripper_com_vector(0), 
        -schunk_gripper_com_vector(1), schunk_gripper_com_vector(0), 0.0;

    // figure out how our end effector is rotated by giving dart the
    // arm values and the imu/waist values
    update_dart_state();
    Eigen::Matrix3d ee_rotation = end_effector_node->getWorldTransform().topLeftCorner<3,3>().transpose();
	
    // Create the wrench with computed rotation to change the frame
    // from the world to the sensor
    Eigen::Matrix6d wrenchrotate_sensor_world = Eigen::MatrixXd::Identity(6,6); 
    wrenchrotate_sensor_world.topLeftCorner<3,3>() = ee_rotation;
    wrenchrotate_sensor_world.bottomRightCorner<3,3>() = ee_rotation;
	
    // Get the weight vector (note that we use the world frame for
    // gravity so towards -y)
    Eigen::Vector6d end_effector_weight_in_world;
    end_effector_weight_in_world << 0.0, end_effector_mass * -9.81, 0.0, 0.0, 0.0, 0.0;
	
    // Compute what the force and torque should be without any
    // external values by multiplying the position and rotation
    // transforms with the expected effect of the gravity
    Eigen::Vector6d expected_ft_reading = transform_eecom_sensor * wrenchrotate_sensor_world * end_effector_weight_in_world;

    // DISPLAY_VECTOR(expected_ft_reading);

    // Remove the effect from the sensor value, convert the wrench
    // into the world frame, and return the result
    Eigen::Vector6d external = input - expected_ft_reading;
    external = wrenchrotate_sensor_world.transpose() * external;
    return external;
}

// #############################################################################
// #############################################################################
// UPDATE STATE
// #############################################################################
// #############################################################################

double get_imu() {
    // Get a message
    int r;
    struct timespec timeout = aa_tm_future(aa_tm_sec2timespec(1.0/30.0));
    Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
                                                        &protobuf_c_system_allocator,
                                                        IMU_CHANNEL_SIZE,
                                                        &imu_chan,
                                                        &timeout);
    assert((imu_msg != NULL) && "Didn't get IMU message!");

    // extract the data into something we can use
    double imu_sample_x  = imu_msg->data[0];
    double imu_sample_y  = imu_msg->data[1];
    double imu_sample_z  = imu_msg->data[2];

    // Free the unpacked message
    somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

    // compute our result and return it
    return -ssdmu_pitch(imu_sample_x, imu_sample_y, imu_sample_z) + M_PI/2;				 
}

Eigen::Vector6d get_ft() {
    // get a message
    int r;
    size_t num_bytes = 0;
    struct timespec timeout = aa_tm_future(aa_tm_sec2timespec(1.0/100.0));
    Somatic__ForceMoment* ft_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__force_moment,
                                                            &protobuf_c_system_allocator,
                                                            1024,
                                                            &ft_chan,
                                                            &timeout);
    assert((ft_msg != NULL) && "Didn't get FT message!");

    // extract the data into something we can use
    Eigen::Vector6d ft_reading;
    for(size_t i = 0; i < 3; i++) ft_reading(i) = ft_msg->force->data[i];
    for(size_t i = 0; i < 3; i++) ft_reading(i+3) = ft_msg->moment->data[i];

    // free the unpacked message
    somatic__force_moment__free_unpacked(ft_msg, &protobuf_c_system_allocator);
    
    // and return our result
    return ft_reading;
}

void update_motor_state() {
    // Read the waist's state and update the averaged waist position
    somatic_motor_update(&daemon_cx, &waist);
    waist_angle = (waist.pos[0] - waist.pos[1]) / 2.0;

    // read the arm
    somatic_motor_update(&daemon_cx, &rlwa);
}

void update_dart_state() {
    // update the waist and imu in dart
    Eigen::VectorXd imu_pos(1);
    imu_pos << imu_angle;
    krang->setConfig(imu_ids, imu_pos);

    Eigen::VectorXd waist_pos(1);
    waist_pos << waist_angle;
    krang->setConfig(waist_ids, waist_pos);

    // update the arm
    Eigen::VectorXd arm_pos(7);
    for(int i = 0; i < 7; i++) arm_pos[i] = rlwa.pos[i];
    krang->setConfig(rarm_ids, arm_pos);
}

// #############################################################################
// #############################################################################
// MAIN LOOP
// #############################################################################
// #############################################################################

void run() {
    // controller variables
    double rcur_command[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // force compliance variables
    Eigen::Vector6d r_ft_raw;
    Eigen::Vector6d r_ft_corrected;
    Eigen::Vector6d r_ft_external;
    Eigen::Vector7d r_arm_vels;
    for(int i = 0; i < 7; i++) r_arm_vels[i] = 0.0; // in case we skip something important

    // loop variables
    double last_ft_update_time = gettime();
    double last_display_time = gettime();
    double last_pid_time = gettime();
    double now;
    
    // tell the user we're going
    std::cout << std::endl << "running!" << std::endl;
    
    while(!somatic_sig_received) {
        // update our time
        now = gettime();
        
        // update our motors
        update_motor_state();
        
        // update DART from those
        update_dart_state();

        // update the IMU and FT sensor, but don't do it too often
        // because those functions block until they've gotten data.
        if (now - last_ft_update_time > 1.0 / ft_update_freq) {
            last_ft_update_time = now;
            // imu_angle = get_imu();

            // if (now - last_display_time > 1.0 / display_freq) {
            //     std::cout << std::setprecision(5);
            //     DISPLAY_VECTOR(r_ft_raw);
            //     DISPLAY_VECTOR(r_ft_corrected);
            // }

            r_ft_raw = get_ft();
            r_ft_corrected = r_ft_raw + r_ft_offset;
        }
        
        // figure out the external forces at this tick
        r_ft_external = compute_external_force(r_ft_corrected);

        // threshold the external values.
        if ((r_ft_external.topLeftCorner<3,1>().norm() < compliance_threshold_force) &&
            (r_ft_external.bottomLeftCorner<3,1>().norm() < compliance_threshold_torque)) {
            // we're under the threshold, so ignore it and send zero current
            for(int i = 0; i < 7; i++) r_arm_vels[i] = 0.0;

            // TODO: add gravity compensation here
        }
        else {
            // we're above the threshold, so figure out what we need
            // to send to comply
            
            // first, increase our sensitivity to torques
            r_ft_external(3) *= 20;
            r_ft_external(4) *= 20;
            r_ft_external(5) *= 40;
            
            // figure out our necessary velocity
            r_arm_vels = wrench_to_joint_vels(r_ft_external);
        }

        // now we figure out where to go
        // TODO: convert vels to currents
        if (do_send_cmds) {
            // cheating way: just send velocity command raw
            // somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY, r_arm_vels.data(), 7, NULL);

            // simple way: integrate velocity to get position, then
            // use PID on current to achieve position and velocity.
            for(int i = 0; i < 7; i++) {
                rpids[i].vel_target = r_arm_vels[i];
                rpids[i].pos_target += (now - last_pid_time) * r_arm_vels[i];
            }
            update_pids(&rlwa, rpids, rcur_command);
            somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, rcur_command, 7, NULL);

            // right way: use the jacobian to map necessary forces and
            // torques to joint torques, then map joint torques to
            // currents.

            last_pid_time = now;
        }

        if (now - last_display_time > 1.0 / display_freq) {
            last_display_time = now;
            std::cout << std::setprecision(5);

            Eigen::Vector7d rpos_setpoint;
            Eigen::Vector7d rpos_current;
            for(int i = 0; i < 7; i++) {
                rpos_setpoint[i] = rpids[i].pos_target;
                rpos_current[i] = rlwa.pos[i];
            }
            
            DISPLAY_VECTOR(r_ft_external);
            DISPLAY_VECTOR(r_arm_vels);
            DISPLAY_VECTOR(rpos_current);
            DISPLAY_VECTOR(rpos_setpoint);
            std::cout << std::endl;
        }
    }
}

// #############################################################################
// #############################################################################
// INIT
// #############################################################################
// #############################################################################

void init() {
    // init dart and figure out our mappings. we do this _before_ we
    // initialize our daemon because somatic_d_init will change our
    // working directory to the place where somatic logs go.
    DartLoader dl;
    world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
    assert((world != NULL) && "Could not find the world");
    krang = world->getSkeleton(0);
    end_effector_node = krang->getNode(end_effector_node_name);

    for(int i = 0; i < 7; i++) rarm_ids.push_back(rarm_ids_initializer[i]);
    for(int i = 0; i < 1; i++) waist_ids.push_back(waist_ids_initializer[i]);
    for(int i = 0; i < 1; i++) imu_ids.push_back(imu_ids_initializer[i]);

    // init daemon
    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
    daemon_opt.ident = "arm-current-compliance";
    somatic_d_init(&daemon_cx, &daemon_opt);

    // init waist
    somatic_motor_init(&daemon_cx, &waist, 2, "waist-cmd", "waist-state");

    // init right arm
    somatic_motor_init(&daemon_cx, &rlwa, 7, "rlwa-cmd", "rlwa-state");
    if (do_send_cmds) {
        somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    }
    usleep(1e5);                // wait 100 ms for it to get going

    // set arm joint limits
    double** arm_minimum_values[] = { &rlwa.pos_valid_min, &rlwa.vel_valid_min,
                                      &rlwa.pos_limit_min, &rlwa.vel_limit_min };
    double** arm_maximum_values[] = { &rlwa.pos_valid_max, &rlwa.vel_valid_max,
                                      &rlwa.pos_limit_max, &rlwa.vel_limit_max };
    for(size_t i = 0; i < 4; i++) aa_fset(*arm_minimum_values[i], -1024.1, 7);
    for(size_t i = 0; i < 4; i++) aa_fset(*arm_maximum_values[i], 1024.1, 7);
    
    // set waist joint limits
    double** waist_minimum_values[] = { &waist.pos_valid_min, &waist.vel_valid_min,
                                        &waist.pos_limit_min, &waist.vel_limit_min };
    double** waist_maximum_values[] = { &waist.pos_valid_max, &waist.vel_valid_max,
                                        &waist.pos_limit_max, &waist.vel_limit_max };
    for(size_t i = 0; i < 4; i++) aa_fset(*waist_minimum_values[i], -1024.1, 2);
    for(size_t i = 0; i < 4; i++) aa_fset(*waist_maximum_values[i], 1024.1, 2);

    // Open the ach channels for the IMU and FT sensor
    somatic_d_channel_open(&daemon_cx, &imu_chan, "imu-data", NULL);
    somatic_d_channel_open(&daemon_cx, &ft_chan, "rlwa_ft", NULL);

    // start the daemon running
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    // update our motors
    update_motor_state();

    // Get a nice, clean IMU reading and hang on to it
    std::cout << "Getting initial IMU reading" << std::endl;
    imu_angle = 0.0;
    for(int i = 0; i < imu_init_iters; i++) { imu_angle += get_imu(); }
    imu_angle /= (double)imu_init_iters;

    // Get a nice, clean force-torque reading
    std::cout << "Getting initial force-torque reading" << std::endl;
    Eigen::Vector6d ft_raw_initial_reading;
    ft_raw_initial_reading << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    for(int i = 0; i < ft_init_iters; i++) { ft_raw_initial_reading += get_ft(); }
    ft_raw_initial_reading /= (double)ft_init_iters;

    // and use it to compute the ft sensor's offset
    r_ft_offset = compute_ft_offset(ft_raw_initial_reading);

    // display the computation of the offset
    DISPLAY_VECTOR(ft_raw_initial_reading);
    DISPLAY_VECTOR(r_ft_offset);

    // initialize controllers
    do_init_pids(&rlwa, rpids);
    for(int i = 0; i < 7; i++) {
        rpids[i].K_p_p = init_K_p_p[i];
        rpids[i].K_p_d = init_K_p_d[i];
        rpids[i].K_v_p = init_K_v_p[i];
        rpids[i].K_v_d = init_K_v_d[i];
        rpids[i].use_pos = use_pos[i];
        rpids[i].use_vel = use_vel[i];
    }
}

// #############################################################################
// #############################################################################
// DESTROY
// #############################################################################
// #############################################################################

void destroy() {
    // Send stopping event
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // halt and close motors as appropriate
    if (do_send_cmds) {
        somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
    }
    somatic_motor_destroy(&daemon_cx, &rlwa);
    somatic_motor_destroy(&daemon_cx, &waist);

    // Close imu channel
    somatic_d_channel_close(&daemon_cx, &imu_chan);

    // Destroy the daemon resources
    somatic_d_destroy(&daemon_cx);
}

// #############################################################################
// #############################################################################
// ENTRY POINT
// #############################################################################
// #############################################################################

int main(int argc, char* argv[]) {
    init();
    run();
    destroy();
}