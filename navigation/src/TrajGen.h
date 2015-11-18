/*
 * Author : Nehchal J. (nehchal@gatech.edu)
 * Georgia Institute of Technology
 *
 * Description: Trajectory Generator implementing trapezoidal motion profile.
 * 
 * Reference:
 *      "Design Trends" Motion Designs Inc. Aug, 2010.
 *      URL: <http://www.motion-designs.com/images/DTrends_Aug_2010.pdf>
 *      Last Reteived: Nov 17, 2015
 */

 #ifndef TRAJ_GEN
 #define TRAJ_GEN

#include <Eigen/Dense>

typedef Eigen::Matrix<double,6,1> Vector6d;

class TrajGen {
    /* State is a 6-vector (x, y, theta, xDot, yDot, thetaDot) */
public:
    TrajGen();

    /* Set the Target Pose
        start_time: [IN] the time when motion starts
        pose: [IN] target pose */
    void setTargetState(const Vector6d& start_state, const Vector6d& target_state);

    /* Returns the reference pose at time t
     t : [IN] current time (in secs) */
    Vector6d getReferenceState(double t);

private:
    double _start_time;

    // The parameters of the trapezoidal motion profile
    double _T_a; // acceleration time
    double _T_d; // deceleration time
    double _T;   // Time taken to reach target
    double _sDot_max;    // Maximum velocity

    Vector6d _start_state;
    Vector6d _end_state;

    // Linear limits of the robot
    double _vel_avg_lin;    // m/sec

    // Rotational limits of the robot
    double _vel_avg_rot;    // rad/sec
};


 #endif