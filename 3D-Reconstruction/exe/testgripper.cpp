// Author: Xinyan Yan
// Date: 11/20
// Test robotiq gripper open and close.
// Press 'o' to open, 'n' to middle open, 'c' to close

#include <kore.hpp>
#include <kore/util.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <somatic/msg.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

using namespace std;

// For input command from keyboard
static int tty_unbuffered(int);     
static void tty_atexit(void);
static int tty_reset(int);
static void tweak_init();

// Print out current state
static void printCurPosCont(double current[], double position[], bool contact[]);

static struct termios save_termios;
static int  ttysavefd = -1;


// Constants
const double DISPLAY_FREQUENCY = 50.0;
const double LOOP_FREQUENCY = 200.0;
const Krang::Side sde = Krang::RIGHT;
const double MAX_CLOSE_TIME = 2;
const double NEUTRAL_POSI[] = {70,70,70,128};
const double CLOSE_POSI[] = {255,255,255,128};
const double OPEN_POSI[] = {0,0,0,128};


//vars

simulation::World* world;
dynamics::SkeletonDynamics* robot;

Krang::Hardware* hw;                                   ///< connects to hardware

somatic_d_t daemon_cx;                          ///< daemon context

double current_threshold = 18;

// debug information
bool debug_print_this_it;       ///< whether we print


void init() {

	// Initalize dart. Do this before we initialize the daemon because initalizing the daemon 
	// changes our current directory to somewhere in /var/run.
	DartLoader dl;
	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton("Krang");

	// create daemon context
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "xinyan";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// Initialize the hardware
	Krang::Hardware::Mode mode = (Krang::Hardware::Mode)(Krang::Hardware::MODE_ALL);
	hw = new Krang::Hardware(mode, &daemon_cx, robot);

	// Start the daemon_cx running
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Init keyboard input feature
    tweak_init();

	
}

void run() {

	uint8_t *buff;

	// Current current and position
	double current[] = {0,0,0,0};
	double position[] = {0,0,0,0};

	// Fingers in contact
	bool contact[] = {false, false, false, false};

	// Whether to send a new command because of new finter(s) are in contact
	bool tostop = false;		

	// Whether to output some termination info and finish this move
	bool toterm = true;

	// Whether in move mode
	bool move = true;

	// For storing command from keyboard
	char c;

	// Desired finger position to achieve
	double goal[4];
	for (int i = 0; i < 4; i++)
		goal[i] = NEUTRAL_POSI[i];

	// Largest current in this move
	double largecurrent[] = {0,0,0,0};

//	double goal[] = {10,10,10,128};
//	double goal[] = {10,10,10,128};
//	double goal[] = {240,240,240,128};


	// Init position
	somatic_motor_setpos(&daemon_cx, hw->grippers[sde], goal, 4);

	// Start some timers
	double time_last_display = aa_tm_timespec2sec(aa_tm_now());
	double time_last = aa_tm_timespec2sec(aa_tm_now());
	double time_accumulated = 0;

	while(!somatic_sig_received) {

		// New command from keyboard?
		// Reset goal
		if ( read(STDIN_FILENO, &c, 1) == 1) {
		    switch (c) {
		        case 'n':
					cout << "####################" << endl;
					cout << "Approaching NEUTRAL" << endl;
					move = true;
					for (int i = 0; i < 4; i++)
						goal[i] = NEUTRAL_POSI[i];
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], goal, 4);
		            break;
		        case 'c':
					cout << "####################" << endl;
					cout << "Approaching CLOSE" << endl;
					move = true;
					for (int i = 0; i < 4; i++)
						goal[i] = CLOSE_POSI[i];
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], goal, 4);
		            break;
		        case 'o':
					cout << "####################" << endl;
					cout << "Approaching OPEN" << endl;
					move = true;
					for (int i = 0; i < 4; i++)
						goal[i] = OPEN_POSI[i];
					somatic_motor_setpos(&daemon_cx, hw->grippers[sde], goal, 4);
		            break;
				case 'u':
					current_threshold += 1;
					cout << "Current threshold: " << current_threshold << endl;
					break;

				case 'd':
					current_threshold -= 1;
					cout << "Current threshold: " << current_threshold << endl;
					break;


				default:
					cout << "Unknown keyboard input" << endl;
		    }
		}
                    
                    
		// Update timers
		double time_now = aa_tm_timespec2sec(aa_tm_now());
		double time_delta = time_now - time_last;
		time_last = time_now;
//		cout << "time accumulated: " << time_accumulated << endl;


		// In move mode?
		if (move) {

			time_accumulated += time_delta;
	
			// Run out of time?
			if (time_accumulated > MAX_CLOSE_TIME) {
				cout << "Finish approaching ABNORMALLY." << endl;
				// Print out current state
				printCurPosCont(current, position, contact);
				move = false;
			}
	
			// Update sensors
			hw->updateSensors(time_delta);
			somatic_motor_update(&daemon_cx, hw->grippers[sde]);
	

			// Read out current and position
			for (int i = 0; i < 4; i++) {
				current[i] = hw->grippers[sde]->cur[i];
				position[i] = hw->grippers[sde]->pos[i];
				if (current[i] > largecurrent[i])
					largecurrent[i] = current[i];
			}
	
	
			// Check termination condition
			for (int i = 0; i < 4; i++) {
				// Is this finger still in process?
				if ( (position[i] != goal[i]) && (contact[i] == false)  )
					toterm = false;
			}
	
			if (toterm) {
				cout << "Finish approaching SUCCESSFULLY. Fingers in contact: ";
				for (int i = 0; i < 4; i++) {
					if (contact[i] == true) {
						cout << i+1 << "   ";
					}
				}
				cout << endl;
				// Print current state
				printCurPosCont(current, position, contact);

				// Print max current
				cout << "Largest current: ";
				for (int i = 0; i < 4; i++)
					cout << largecurrent[i] << ", ";
				cout << endl;

				// Reset some vars
				move = false;
				time_accumulated = 0;
				for (int i = 0; i < 4; i++) {
					contact[i] = false;
					largecurrent[i] = 0;
				}



			} else {
				// Reset toterm
				toterm = true;
			}
	
	
			// Check current
			for (int i = 0; i < 4; i++) {
	
				if ( (current[i] >= current_threshold) && (contact[i] == false) ) {
	
					cout << "Found new finger in contact: " << i+1 << endl;
					contact[i] = true;
					// Print current state
					printCurPosCont(current, position, contact);

				}
			}
	
	
			// Reset the goal
			if (tostop) {
				cout << "Reset goal to ";
				for (int i = 0; i < 4; i++) 
					cout << goal[i] << "   ";
				cout << endl;
	
				somatic_motor_setpos(&daemon_cx, hw->grippers[sde], goal, 4);
				// Reset tostop
				tostop = false;
			}
	
	
	//		// Achieve goal?
	//		for (int i = 0; i < 4; i++) {
	//			if ( abs(hw->grippers[sde]->pos[i] - goal[i]) > 1) {
	//				achieved = false;
	//			}
	//		}
	//		if (achieved == true) {
	//			cout << "Achieved the goal!" << endl;
	//			break;
	//		} else {
	//			achieved = true;
	//		}
				
	
			// set up debug printing
			debug_print_this_it = (time_now - time_last_display) > (1.0 / DISPLAY_FREQUENCY);
			if(debug_print_this_it) time_last_display = time_now;
	
			if (debug_print_this_it) {
//				printCurPosCont(current, position, contact);
			
			}
		}
	
		// And sleep to fill out the loop period so we don't eat the entire CPU
		double time_loopend = aa_tm_timespec2sec(aa_tm_now());
		double time_sleep = (1.0 / LOOP_FREQUENCY) - (time_loopend - time_now);
		int time_sleep_usec = std::max(0, (int)(time_sleep * 1e6));
		
	}
}


/// Clean up
void destroy() {

	// Stop the daemon
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);


	// Close down the hardware
	delete hw;

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

}


int main(int argc, char* argv[]) {

	init();
	run();
	destroy();
}


static void printCurPosCont(double current[], double position[], bool contact[])
{
	// Current
	cout << "Current: ";
	for (int i = 0; i < 4; i++)
		cout << current[i] << ", ";
	cout << endl;

	// Position
	cout << "Position: ";
	for (int i = 0; i < 4; i++)
		cout << position[i] << ", ";
	cout << endl;
	
	// Contact
	cout << "Contact: ";
	for (int i = 0; i < 4; i++)
		cout << contact[i] << ", ";
	cout << endl;

}

static int
tty_unbuffered(int fd)      /* put terminal into a raw mode */
{
    struct termios  buf;

    if (tcgetattr(fd, &buf) < 0)
        return(-1);
        
    save_termios = buf; /* structure copy */

    /* echo off, canonical mode off */
    buf.c_lflag &= ~(ECHO | ICANON);

    /* 1 byte at a time, no timer */
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSAFLUSH, &buf) < 0)
        return(-1);

    ttysavefd = fd;
    return(0);
}

static int
tty_reset(int fd)       /* restore terminal's mode */
{
    if (tcsetattr(fd, TCSAFLUSH, &save_termios) < 0)
        return(-1);
    return(0);
}

static void
tty_atexit(void)        /* can be set up by atexit(tty_atexit) */
{
    if (ttysavefd >= 0)
        tty_reset(ttysavefd);
}

static void
tweak_init()
{
   /* make stdin unbuffered */
    if (tty_unbuffered(STDIN_FILENO) < 0) {
        std::cout << "Set tty unbuffered error" << std::endl;
        exit(1);
    }

    atexit(tty_atexit);

    /* nonblock I/O */
    int flags;
    if ( (flags = fcntl(STDIN_FILENO, F_GETFL, 0)) == 1) {
        perror("fcntl get flag error");
        exit(1);
    }
    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl set flag error");
        exit(1);
    }
}




//
//#include <Hubo_Control.h>
//#include <iostream>
//#include <fstream>
//#include <getopt.h>
//#include "impedanceController.h"
//#include <termio.h>
//#include <stdio.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <stdlib.h>
//
//static int tty_unbuffered(int);     
//static void tty_atexit(void);
//static int tty_reset(int);
//static void tweak_init();
//
//static struct termios save_termios;
//static int  ttysavefd = -1;
//
//double M, Q, K; // best numbers so far: M=0.008, Q=0.4, K=7.0
//double MStep, QStep, KStep;
//
//
///**
// * Prints out how to run this program
//*/
//void usage(std::ostream& ostr) {
//    ostr << 
//        "usage: fastrak-arms [OPTIONS] \n"
//        "\n"
//        "OPTIONS:\n"
//        "\n"
//        "  -l, --left           Control left arm only.\n"
//        "  -r, --right          Control right arm only.\n"
//        "  -n, --nosend         Don't send commands to Hubo.\n"
//        "  -V, --verbose        Show output.\n"
//        "  -H, --help           See this message\n";
//}
//
//int main(int argc, char **argv)
//{
//
//    M = 0.002; // user set inertia (kg)
//    Q = 0.1; // sungmoon changed to .1 (on 4/2/2013) original was .4 user set damping (N-s/m)
//    K = 2.5; //.5;//sungmoon changed to 1 (on 4/2/2013) original was 7; // user set stiffness (N/m)
//
//    MStep = 0.001;
//    QStep = 0.1;
//    KStep = 1.0;
//
//    // check if no arguments given, if not report usage
//    if (argc < 2)
//    {
//        usage(std::cerr);
//        return 1;
//    }
//
//    bool print = false; // whether to print output or not
//    bool left = false; // whether to set left arm angles
//    bool right = false; // whether to set right arm angles
//    bool send = true; // whether to send commands or not
//
//    // command line lone options
//    const struct option long_options[] = 
//    {
//        { "left",       no_argument, 0, 'l' },
//        { "right",      no_argument, 0, 'r' },
//        { "nosend",     no_argument, 0, 'n' },
//        { "verbose",    no_argument, 0, 'V' },
//        { "help",       no_argument, 0, 'H' },
//        { 0,            0,           0,  0  },
//    };
//
//    // command line short options
//    const char* short_options = "lrnVH";
//
//    // command line option and option index number
//    int opt, option_index;
//
//    // loop through command line options and set values accordingly
//    while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 )
//    {
//        switch (opt)
//        {
//            case 'l': left = true; break;
//            case 'r': right = true; break;
//            case 'n': send = false; break;
//            case 'V': print = true; break;
//            case 'H': usage(std::cout); exit(0); break;
//            default:  usage(std::cerr); exit(1); break;
//        }
//    }
//
//    //=== OBJECTS ===//
//    // Create Hubo_Control object
//    Hubo_Control hubo;
////    std::cout << "Daemonizing as impedanceCtrl\n";
////    Hubo_Control hubo("impedanceCtrl");
//
//    //=== LOCAL VARIABLES ===//
//    ArmVector rActualAngles, rArmAnglesCurrent, rArmAnglesNext, checkr;
//    ArmVector lActualAngles, lArmAnglesNext, lArmAnglesCurrent, checkl;
//    ArmVector armNomAcc, armNomVel;
//    Eigen::Isometry3d lcurrEE, rcurrEE, lTransf, rTransf, lHandCurrent, rHandCurrent, wristTF;
//    int i=0, imax=40;
//    double dt, ptime;
//    double lebDesired, lsyDesired, rebDesired, rsyDesired, dMx, dMy;
//    Eigen::Vector2d dqLeb(0,0);
//    Eigen::Vector2d dqLsy(0,0);
//    Eigen::Vector2d dqReb(0,0);
//    Eigen::Vector2d dqRsy(0,0);
//    double a=3.0, v=3.0;
//    double aMax=15.0, vMax=15.0;
//    // Define starting joint angles for the arms 
//    lArmAnglesNext << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//    rArmAnglesNext << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
// 
//    // Set the arm joint angles and send commands to the control daemon
//    if(left == true)
//    {
//        lArmAnglesNext << 0, -.3, 0, -M_PI/2, 0, 0, 0, 0, 0, 0;
//        hubo.setLeftArmAngles( lArmAnglesNext, true );
//    }
//    if(right == true)
//    {
//        rArmAnglesNext << 0, .3, 0, -M_PI/2, 0, 0, 0, 0, 0, 0;
//        hubo.setRightArmAngles( rArmAnglesNext, true );
//    }
//    // While the norm of the right arm angles is greater than 0.075
//    // keep waiting for arm to get to desired position
//    while ((lArmAnglesNext - checkl).norm() > 0.075 || (rArmAnglesNext - checkr).norm() > 0.075)
//    {
//        hubo.update(); // Get latest data from ach channels
//        hubo.getLeftArmAngles(checkl); // Get current left arm joint angles
//        hubo.getRightArmAngles(checkr); // Get current right arm joint angles
//    }
//
//    // Define arm nominal acceleration 
//    armNomAcc << a, a, a, a*5/3, a, a, 0, 0, 0, 0;
//    armNomVel << v, v, v, v*5/3, v, v, 0, 0, 0, 0;
//
//    // Set arm nominal accelerations 
//    hubo.setLeftArmNomAcc(armNomAcc);
//    hubo.setRightArmNomAcc(armNomAcc);
//    hubo.setLeftArmNomSpeeds(armNomVel);
//    hubo.setRightArmNomSpeeds(armNomVel);
//
//    usleep(500000);
//    //double MdLx = hubo.getLeftHandMx();
//    double MdLx = hubo.getLeftHandMy();
//    //double MdLy = hubo.getLeftHandMy();
//    double MdLy = -hubo.getLeftHandMx();
//    double MdRx = hubo.getRightHandMx();
//    double MdRy = hubo.getRightHandMy();
//    // double FdRz = hubo.getRightHandFz(); // added by sungmoon 4/2/2013 --> RightHandFz not defined in Hubo-Control
//    Eigen::Vector3d qNew(0.0, 0.0, 0.0);
//
//    // get current joint angles
//    lebDesired = hubo.getJointAngle(LEB);
//    rebDesired = hubo.getJointAngle(REB);
//    lsyDesired = hubo.getJointAngle(LSY);
//    rsyDesired = hubo.getJointAngle(RSY);
//    ptime = hubo.getTime(); // set initial time for dt(0)
//
//    std::cout << "Executing control loop ...\n";
//
//    tweak_init();
//
//    char c;
//
//    while(!daemon_sig_quit)
//    {
//        // get latest state info for Hubo
//        hubo.update();
//
//        dt = hubo.getTime() - ptime;
//        ptime = hubo.getTime();
//
//        // if new data is available...
//        if(dt > 0)
//        {
//            if ( read(STDIN_FILENO, &c, 1) == 1) {
//                switch (c) {
//                    case 'u':
//                        M += MStep;
//                        break;
//                    case 'j':
//                        if ( M - MStep > 0)
//                            M -= MStep;
//                        if (M < 0.001)
//                            M = 0.001;
//                        break;
//                    case 'i':
//                        Q += QStep;
//                        break;
//                    case 'k':
//                        if ( Q - QStep > 0)
//                            Q -= QStep;
//                        break;
//                    case 'o':
//                        K += KStep;
//                        break;
//                    case 'l':
//                        if ( K - KStep > 0)
//                            K -= KStep;
//                        break;
//                }
//                std::cout << "M: " << M  << "   " << "Q: " << Q << "   " << "K: " << K << std::endl; 
//            }
//
//            // set and get joint angles
//            if( left==true )
//            {
//                // Y-Moment (Elbow)
//                qNew.setZero();
//                dMy = -hubo.getLeftHandMx() - MdLy;
//                qNew = impedanceController(dqLeb, dMy, lebDesired, dt);
//                hubo.setJointAngle( LEB, qNew(0), false );
//               // hubo.setJointNominalSpeed(LEB*v, qNew(1));
//                //hubo.setJointNominalAcceleration(LEB*a, qNew(2));
//                // X-Moment (Shoulder Yaw)
//                qNew.setZero();
//                dMx = hubo.getLeftHandMy() - MdLx;
//                qNew = impedanceController(dqLsy, dMx, lsyDesired, dt);
//                hubo.setJointAngle( LSY, qNew(0), false );
//                //hubo.setJointNominalSpeed(LSY*v, qNew(1));
//                //hubo.setJointNominalAcceleration(LSY*a, qNew(2));
//            }
//
//            if( right==true )
//            {
//                // Y-Moment (elbow)
//                qNew.setZero();
//                dMy = hubo.getRightHandMy() - MdRy;
//                qNew = impedanceController(dqReb, dMy, rebDesired, dt);
//                hubo.setJointAngle( REB, qNew(0), false );
//                // X-Moment (Shoulder yaw)
//                qNew.setZero();
//                dMx = hubo.getRightHandMx() - MdRx;
//                qNew = impedanceController(dqRsy, dMx, rsyDesired, dt);
//                hubo.setJointAngle( RSY, qNew(0), false);
//                a = fabs((hubo.getJointAngle(REB) - hubo.getJointAngleState(REB)));
//                a = (a < 0.2) ? a : 0.2;
//                a = (25*a/0.2 > 15) ? 25*a/0.2 : 15;
//                hubo.setJointNominalSpeed(REB, a);
//                hubo.setJointNominalAcceleration(REB, a);
//            }
//
//            // send control references
//            if( send == true )
//            {
//                hubo.sendControls();
//            }
//
//            // print output every imax cycles
//            if( i>=imax && print==true )
//            {
//                std::cout //<< "\033[2J"
//                          << "LEB-Desired: " << lebDesired*180/M_PI
//                          << "\nLEB-Actual : " << hubo.getJointAngleState(LEB)*180/M_PI
//                          << "\nLeft  hand torques(N-m)(MdX,Mx|MdY,My): " << MdLx << ", " << hubo.getLeftHandMx() << " | " 
//                                                                          << MdLy << ", " << hubo.getLeftHandMy()
//                          << "\nREB-Desired: " << rebDesired*180/M_PI
//                          << "\nREB-qNew   : " << qNew*180/M_PI
//                          << "\nREB-Actual : " << hubo.getJointAngleState(REB)*180/M_PI
//                          << "\nRight hand torques(N-m)(MdX,Mx|MdY,My): " << MdRx << ", " << hubo.getRightHandMx() << " | " 
//                                                                          << MdRy << ", " << hubo.getRightHandMy() //<< " | "
//                          //<< " Fz          : " << hubo.getRightHandFz()
//                          << "\nM: " << M << "\tQ: " << Q << "\tK: " << K
//                          << "\na: " << a
//                          << std::endl;
//            }
//            if(i>=imax) i=0; i++;
//        }
//    }
//    return 0;
//}
//
//    /* 1 byte at a time, no timer */
//    buf.c_cc[VMIN] = 1;
//    buf.c_cc[VTIME] = 0;
//    if (tcsetattr(fd, TCSAFLUSH, &buf) < 0)
//        return(-1);
//
//    ttysavefd = fd;
//    return(0);
//}
//
//static int
//tty_reset(int fd)       /* restore terminal's mode */
//{
//    if (tcsetattr(fd, TCSAFLUSH, &save_termios) < 0)
//        return(-1);
//    return(0);
//}
//
//static void
//tty_atexit(void)        /* can be set up by atexit(tty_atexit) */
//{
//    if (ttysavefd >= 0)
//        tty_reset(ttysavefd);
//}
//
//static void
//tweak_init()
//{
//   /* make stdin unbuffered */
//    if (tty_unbuffered(STDIN_FILENO) < 0) {
//        std::cout << "Set tty unbuffered error" << std::endl;
//        exit(1);
//    }
//
//    atexit(tty_atexit);
//
//    /* nonblock I/O */
//    int flags;
//    if ( (flags = fcntl(STDIN_FILENO, F_GETFL, 0)) == 1) {
//        perror("fcntl get flag error");
//        exit(1);
//    }
//    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
//        perror("fcntl set flag error");
//        exit(1);
//    }
//}
//



