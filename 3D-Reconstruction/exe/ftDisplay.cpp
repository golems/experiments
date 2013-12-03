// Author: Xinyan Yan
// Date: 11/15/2013
// Display the force torque sensor reading

#include <kore.hpp>
#include <kore/util.hpp>

#include <iostream>
#include <iomanip>

#include <somatic/msg.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

using namespace std;

simulation::World *world;

dynamics::SkeletonDynamics *robot;

Krang::Hardware *hw;

somatic_d_t daemon_cx;

// main
int main(int argc, char *argv[])
{
    DartLoader dl;
    world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
    assert((world != NULL) && ("Could not find the world"));
    robot = world->getSkeleton("Krang");

    somatic_d_opts_t daemon_opt;
    memset(&daemon_opt, 0, sizeof(daemon_opt));
    daemon_opt.ident = "xinyan";
    somatic_d_init(&daemon_cx, &daemon_opt);

    Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL);
    hw = new Krang::Hardware(mode, &daemon_cx, robot);

    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
                    SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    double arm_velocities[7];
    for (int i = 0; i < 7; i++) arm_velocities[i] = 0.0;
    double time_last = aa_tm_timespec2sec(aa_tm_now());

    while (!somatic_sig_received) {
        double time_now = aa_tm_timespec2sec(aa_tm_now());
        double time_delta = time_now - time_last;

        time_last = time_now;
        hw->updateSensors(time_delta);

		for (int i = 0; i < 6; i++)
			cout << hw->fts[Krang::RIGHT]->lastExternal[i] << ", ";
		cout << endl;
			
//        for (int i = 0; i < 7; i++)
//           cout << hw->arms[Krang::LEFT]->pos[i] << ", " << endl;

//        somatic_motor_setvel(&daemon_cx, hw->arms[Krang::LEFT], arm_velocities, 7);
        usleep(300000);
    }

    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
                    SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

}



