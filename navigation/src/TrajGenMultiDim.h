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

 #ifndef TRAJ_GEN_MULTI_DIM
 #define TRAJ_GEN_MULTI_DIM

#include <Eigen/Dense>

#include <TrajGen.h>

typedef Eigen::Matrix<double,6,1> Vector6d;

class TrajGenMultiDim {
    /* State is a 6-vector (x, y, theta, xDot, yDot, thetaDot) */
public:
    TrajGenMultiDim();

    /* Set the Target Pose
        pose: [IN] target pose */
    void setTargetState(const Vector6d& start_state, const Vector6d& target_state);

    /* Returns the reference pose at time t
     t : [IN] time elaspsed since trajectory was set (in secs) */
    Vector6d getReferenceState(double t);

    /* Returns total time of trajectory */
    double getTotalTime();

private:

    TrajGen _trajGen_x;
    TrajGen _trajGen_y;
    TrajGen _trajGen_theta;

    //double _start_time;

    // The parameters of the trapezoidal motion profile
    //double _T_a; // acceleration time
    //double _T_d; // deceleration time
    double _T;   // Time taken to reach target
    //double _sDot_max;    // Maximum velocity

    Vector6d _start_state;
    //Vector6d _end_state;

    // Linear limits of the robot
    double _vel_avg_lin;    // m/sec

    // Rotational limits of the robot
    double _vel_avg_rot;    // rad/sec
};


 #endif // TRAJ_GEN_MULTI_DIM