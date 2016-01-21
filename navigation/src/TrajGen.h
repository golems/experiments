/*
 * Author : Nehchal J. (nehchal@gatech.edu)
 * Georgia Institute of Technology
 *
 * Description: Trajectory Generator implementing trapezoidal motion profile
 *              for one dimension.
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

    /* Set the trajectory. Assumes initial position to be zero
        start_pos: [IN] starting position
        start_vel: [IN] starting velocity
        target_pos: [IN] target position 
        t : [IN] total time. Should be positive */
    void initializeTrajectory(float v_i, float D, float T);

    /* Returns the reference pose at time t
     t : [IN] time elaspsed since trajectory was set (in secs) */
    void getReferenceState(double t, float& x_ref, float& v_ref);

    /* Returns total time of trajectory */
    double getTotalTime();

private:
    double _start_time;

    // The parameters of the trapezoidal motion profile
    double _T_a; // acceleration time
    double _T_d; // deceleration time
    double _T;   // Time taken to reach target
    double _v_cruise;    // Velocity during cruising phase

    float _v_i; // initial velocity
};


 #endif // TRAJ_GEN