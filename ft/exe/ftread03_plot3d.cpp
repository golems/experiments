/**
 * @file ftread03_plot3d.cpp
 * @author Can Erdogan
 * @date Dec 11, 2012
 * @brief This program plots the force and torque measurements in a 3D OpenGL scene.
 * @todo Visualize the torques.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>
#include <algorithm>

#include <math.h>
#include <GL/glut.h>

using namespace std;

// The somatic context and options
somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;

// The ach channel and its name
ach_channel_t achChannel;
const char *channelName = NULL;

// Sampling option
size_t samplingRate = 0;

// Plotting variables
int win;												///< The OpenGL window descriptor
int width, height;							///< The dimensions of the OpenGL window
GLfloat objectRotX, objectRotY;	///< The rotation due to mouse
int curx, cury;									///< Mouse location
float vx = 0, vy = 0, vz = 0;

/* ********************************************************************************************* */
// argp options
static struct argp_option options[] = { 
	{"chan", 'c', "ach channel", 0, "ach channel to receive the force/torque data from"}, 
	{"sample", 's', "sampling rate", 0, "every nth value will be plotted to the OpenGL scene"}, 
	{0} 
};

/* ********************************************************************************************* */
// argp constants
static int parse_opt( int key, char *arg, struct argp_state *state);
static char args_doc[] = "This program prints the force/torque measurements read from a channel";
static struct argp argp = {options, parse_opt, args_doc, NULL, NULL, NULL, NULL };

/* ********************************************************************************************* */
/// The parsing function that argp uses along with the given options
static int parse_opt( int key, char *arg, struct argp_state *state) {
	(void) state; // ignore unused parameter
	switch(key) {
	case 'c':
		if(strlen(arg) > ACH_CHAN_NAME_MAX) {
			fprintf(stderr, "ERROR: channel is too long\n");
			exit(1);
		} else {
			channelName = strdup(arg);
		}
		break;
	case 's':
		samplingRate = atoi(arg);
		break;
	}
	return 0;
}

/* ********************************************************************************************* */
void init() {

	// Set OpenGL options
	glMatrixMode(GL_PROJECTION);
  glFrustum(-0.50, 0.50, -0.50, 0.50, 1.0, 14.0);
  glMatrixMode(GL_MODELVIEW);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

	// Initialize the somatic context 
	// NOTE: somatic_d_init sends a SOMATIC__EVENT__CODES__PROC_STARTING message on the ach channel.
	// NOTE: somatic_d_channel opens the ach channel to read from in update(). 
	somatic_d_init(&somaticContext, &somaticOptions);
	somatic_d_channel_open(&somaticContext, &achChannel, channelName, NULL);
}

/* ********************************************************************************************* */
// @todo Clean up this function
void drawCylinder(int numMinor, float height, float radius) {

  double majorStep = height;
  double minorStep = 2.0 * M_PI / numMinor;
  int j;
  GLfloat z0 = 0.5 * height;
  GLfloat z1 = z0 - majorStep + height * 0.5;
  glBegin(GL_TRIANGLE_STRIP);
  for (j = 0; j <= numMinor; ++j) {
    double a = j * minorStep;
    GLfloat x = radius * cos(a);
    GLfloat y = radius * sin(a);
    glNormal3f(x / radius, y / radius, 0.0);
    glVertex3f(x, y, z0);
    glNormal3f(x / radius, y / radius, 0.0);
    glVertex3f(x, y, z1);
  }
  glEnd();
}

/* ********************************************************************************************* */
void getAngles (float x, float y, float z, float& xAngle, float& yAngle) {

  // Compute the latitude
  if((fabs(x) < 1e-5) && (fabs(y) < 1e-5)) xAngle = 0.0;
  else xAngle = atan2(y, x) / M_PI * 180.0;

  // Compute the elevation
  float temp = x*x + y*y;
  if((fabs(z) < 1e-5) && (fabs(sqrt(temp)) < 1e-5)) yAngle = 0.0;
  else yAngle = atan2(z,sqrt(temp)) / M_PI * 180.0;
}

/* ********************************************************************************************* */
void display (void) {
	
	// Constants
	static const float axisRadius = 0.05;

  // Clear the buffer with the black background
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
	glTranslatef(0.0, -0.2, -3.0);


  // Rotate with the mouse along with the initial fixed rotation
  glPushMatrix();
  glRotatef(objectRotY, 0, 1, 0);
  glRotatef(objectRotX, 1, 0, 0);
  glRotatef(-26, 0, 1, 0);
  glRotatef(-65, 1, 0, 0);

  // Draw the three axis along with their labels
  glColor3f(0.0, 0.0, 1.0);
  drawCylinder(8, 2.0, axisRadius);
  glRasterPos3f(0.0, 0.0, 1.1);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'z');
  glRotatef(90, 0, 1, 0);
  glColor3f(0.0, 1.0, 0.0);
  drawCylinder(8, 2.0, axisRadius);
  glRasterPos3f(0.0, 0.0, 1.1);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'y');
  glRotatef(90, 1, 0, 0);
  glColor3f(1.0, 0.0, 0.0);
  drawCylinder(8, 2.0, axisRadius);
  glRasterPos3f(0.0, 0.0, 1.1);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'x');
	
  // Find the correct rotations and length
  float xAngle, yAngle;
  getAngles(vx, vy, vz, xAngle, yAngle);
  // cout << "vector: " << vx << " " << vy << " " << vz << endl;
  // cout << "angles: " << xAngle << " " << yAngle << endl;

  // Draw the vector
  glRotatef(-xAngle, 1, 0, 0); 
  glRotatef(-yAngle, 0, 1, 0); 
  glColor3f(0.0, 1.0, 1.0);
  if(!((fabs(vx) < 1e-5) && (fabs(vy) < 1e-5) && (fabs(vz) < 1e-5)))
    drawCylinder(8, 2, axisRadius);
  glPopMatrix();
	
	// Update the visualized buffer
  glutSwapBuffers();
}

/* ********************************************************************************************* */
void motion(int x, int y) {
  GLfloat dx = (y - cury) * 360.0 / height;
  GLfloat dy = (x - curx) * 360.0 / width;
  objectRotX += dx;
  objectRotY += dy;
  curx = x;
  cury = y;
  glutPostRedisplay();
}

/* ********************************************************************************************* */
void mouse(int button, int state, int x, int y) {
  if (state == GLUT_DOWN) {
    switch (button) {
      case GLUT_LEFT_BUTTON:
        motion(curx = x, cury = y);
        break;
    }
  }
}

/* ********************************************************************************************* */
void update(void) {

	// Check for an interrupt
	if(somatic_sig_received) throw "need to exit";
	
	// =======================================================
	// A. Get message
	// NOTE: This is usually done with SOMATIC_D_GET which is a macro.

	// Set the time to read (?)
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&somaticContext, &achChannel, &numBytes, &abstimeout, ACH_O_WAIT, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return;

	// =======================================================
	// B. Read message

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(somaticContext.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return;
	if(msg->meta->type != SOMATIC__MSG_TYPE__FORCE_MOMENT) return;

	// Consider skipping the iteration if sampling
	static int counter = 0;
	counter++;
	if((samplingRate != 0) && ((counter % samplingRate) != 0)) return;

	// Read the force-torque message
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(somaticContext.pballoc), numBytes, buffer);

	// =======================================================
	// C. Visualize the forces

	// Compute the normalized value of the forces
	float fx = ftMessage->force->data[0];
	float fy = ftMessage->force->data[1];
	float fz = ftMessage->force->data[2];
	float norm = sqrt(fx * fx + fy * fy + fz * fz);
	if(fabs(norm) >= 1e-4) {
		vx = fx / norm, vy = fy / norm, vz = fz / norm;
	  glutPostRedisplay();
	}

	// =======================================================
	// D. Complete

	// free buffers allocated during this cycle
	aa_mem_region_release(&somaticContext.memreg);	
}

/* ********************************************************************************************* */
void destroy() {

	// Close the channel and end the daemon
	somatic_d_channel_close(&somaticContext, &achChannel);
	somatic_d_destroy(&somaticContext);
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

	// Parse the arguments and make sure a channel name is given
	argp_parse (&argp, argc, argv, 0, NULL, NULL);
	if(channelName == NULL) {
		printf("Usage: ./ftread03_plot3d -c <ach channel name>\n");
		exit(EXIT_FAILURE);
	}

	// Set the somatic context options
	somaticOptions.ident = "ftread03_plot3d";
	somaticOptions.sched_rt = SOMATIC_D_SCHED_NONE; // logger not realtime
	somaticOptions.skip_mlock = 1; 		// not sure why?

	// Set the OpenGL options
  glutInit(&argc, argv);
  glutInitWindowSize(width = 480, height = 480);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  win = glutCreateWindow("Force Direction");
  glutDisplayFunc(display);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
	glutIdleFunc(update);
	
	// Initialize the somatic context and the OpenGL options
  init();

	// Send the starting message; set the event code and the priority
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Run the glut main loop
	// NOTE: This is a hack to break out of the glut main loop
	try {
	  glutMainLoop();
	} catch (const char* msg) {
	}

	// Send the stopping event
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE,
			SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Destroy the context
	destroy();

	exit(EXIT_SUCCESS);
}
