/**
 * @file krang.cpp
 * @author Kasemsit Teeyapan, Can Erdogan, Munzir Zafar, Kyle Volle
 * @date Aug 15, 2010
 * @brief The interface to talk with the main class that represents the state and the modules
 * of Krang. 
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include <amino.h>
#include <schkin.h>
#include "krang.h"
#include "krang-io.h"

/* ********************************************************************************************** */
Krang::Krang(krang_state_t* _state) {

	state = _state;

	// Initialize the path to files that contain position offsets and last positions before Shutdown
	sprintf(positionOffsetsPath, "/var/krang/positionOffsets.bin");
	sprintf(lastPositionsPath, "/var/krang/lastPositions.bin");

	// Initialize names of joint positions 
	sprintf(position[0].str,"Imu             ");
	sprintf(position[1].str,"Waist           ");
	sprintf(position[2].str,"Torso           ");
	for(int i=0;i<7;i++) {
		sprintf(position[3+i].str,"Left Arm Motor %d", i+1);
		sprintf(position[10+i].str,"Right Arm Motor %d", i+1);
	}
	// Initialize the kalman filter
	kf = new filter_kalman_t;
	filter_kalman_init( kf , 8 , 0 , 8 );

	// Create the interfaces for the joystick, imu, amc and schunk modules
	js	= new Joystick("joystick-data");
	imu   = new Imu("imu-data", 0.0);
	amc   = new Motor("amc-cmd", "amc-state", "AMC", 2);
	waist = new Motor("waist-cmd", "waist-state", "Waist", 2);
	torso = new Motor("torso-cmd", "torso-state", "Torso", 1);
	rlwa  = new Motor("rlwa-cmd", "rlwa-state", "Right arm", 7);
	llwa  = new Motor("llwa-cmd", "llwa-state", "Left arm", 7);

	// Set the minimum and maximum position of the waist modules
	aa_fset( waist->motor.pos_valid_min, -5, 2 );
	aa_fset( waist->motor.pos_valid_max, 5, 2 );

	// Reset PCIO
	waist->reset();
	//torso->reset();
	rlwa->reset();
	llwa->reset();

	// Set the offsets
  // NOTE: We set waist position at the joint limit to zero. The value 0.4889 [rad] is the 
	// angle where all three robot axes are aligned.
/*	double k = -90.0 / 180.0 * M_PI; 
	double waist_offset_val[2] = { -k, k};  
	double rlwa_offset[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double llwa_offset[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double torso_offset[1] = { 0.0 };
	double amc_offset[2] = { -amc->get_pos(0)-imu->get_th(), -amc->get_pos(1)-imu->get_th() }; 
	waist->set_pos_offset(waist_offset_val);
	rlwa->set_pos_offset(rlwa_offset);
	llwa->set_pos_offset(llwa_offset);
	torso->set_pos_offset(torso_offset);
	amc->set_pos_offset(amc_offset);

	// Get an update from interfaces
	usleep(1e5);
	this->update();
*/
	
	while(true) {
		// Read position offset values from file
		this->read_position_offsets_from_file();
		// Read the last values read before the program quit from the file
		this->read_previous_positions_from_file();
		// Update the offsets of the motors
		this->set_position_offsets();
		// Send commands to all the motors to set their velocity to zero, this is only done to 
		// update the values in the ack channels, as they are only updated when a command is sent to the
		// motors
		this->set_velocities_to_zero();
		usleep(1e5);
		// Update all the values
		this->update();
		// FIXME: For some reason imu->get_th() reads zero if this following statement is not added. This 
		// could be pointing to a problem with imu implementation
		//imu->update();
		// Update the position array that lists all the sensor values in sequence. This is only
		// done for ease of display
		this->populate_positions();
		// Display the current values
		fprintf(stderr, "\n\nSANITY CHECK FOR SENSORS:\n (All values in degrees)\n");
		fprintf(stderr, "S.No.\tJoint/Sensor\t\tCurrent Value\t\tValue at Last Exit\tDifference\n");
		for(char i=0; i<17; i++ ) {
			fprintf(stderr, "%c.\t%s\t%.2lf\t\t\t%.2lf\t\t\t%.2lf\n", 
				'a'+i, position[i].str, position[i].current, position[i].last, position[i].last-position[i].current);
		}
		fprintf(stderr, "\n");
		// Ask the user whether the current values make sense
		fprintf(stderr, "\nDo all the current values make sense (y/n)?");
		char c;
		std::cin >> c;
		if( c=='n' ) {
			fprintf(stderr, "Which sensor value doesn't make sense (a-q)?");
			std::cin >> c;
			fprintf(stderr, "For %s the current value is %.2lf, the value at last exit was %.2lf. Difference is %.2lf\n", position[c-'a'].str, position[c-'a'].current, position[c-'a'].last, 
					position[c-'a'].last-position[c-'a'].current);
			fprintf(stderr, "Enter the correct value: ");
			double val;
			std::cin >> val;
			position[c-'a'].offset += val-position[c-'a'].current;
			this->write_position_offsets_to_file();
		}
		else if(c=='y') {
			break;
		}
	}
	

	// Check waist position (module id 14 and 15)
	if (fabs(waist->get_pos(0) + waist->get_pos(1)) > 2e-2)
		fprintf(stderr, "ERROR: Waist position mismatched: (%f, %f)\n", waist->get_pos(0), 
			-waist->get_pos(1));

	// Print initial states
	krang_dump_state(this, 0.0);
}

/* ********************************************************************************************** */
// Read the position measurements from individual motor objects and populate them in the positions
// array
void Krang::populate_positions() {
	// imu
	position[0].current = imu->get_th() * 180.0 / M_PI;
	//std::cout << "imu: " << imu->get_th() << std::endl;
	// waist
	position[1].current = waist->get_pos(0) * 180.0 / M_PI;
	// torso
	position[2].current = torso->get_pos(0) * 180.0 / M_PI;
	// arms
	for(int i=0;i<7;i++) {
		position[3+i].current = llwa->get_pos(i) * 180.0 / M_PI;
		position[10+i].current = rlwa->get_pos(i) * 180.0 / M_PI;
	}
	
}
/* ********************************************************************************************** */
// Read position offsets from file. "Position offsets" are values to be added to the sensed values
// from the imu and schunk module encoders. Saved in degrees.
void Krang::read_position_offsets_from_file() {
	// Open the binary file for reading
  FILE *file =	fopen(positionOffsetsPath, "rb");
	// If the file doesn't exist. Create it and write default values in it.
	if(file == NULL) { 
		// Give error message
		fprintf(stderr, "Could not open position offset file. Creating one with default offset values.\n");
		// Default values of offsets. zeros for all. 90.0 for waist.
		for(int i=0;i<17;i++) {
			position[i].offset = 0.0;
		}
		position[1].offset = 90.0; // Default offset value for the waist module
		// Create a file and write the default offsets in it
		this->write_position_offsets_to_file();
		return;
	}
	
	// If the file exists, read the values of the position offsets
	double val;
	for(int i=0; i<17 ; i++) {
		fread(&position[i].offset, sizeof(double), 1, file);
	}
	printf("lqo: "); for(int i=0; i<7; i++) { printf(" %.2lf", position[3+i].offset);  } printf("\n");
	printf("rqo: "); for(int i=0; i<7; i++) { printf(" %.2lf", position[10+i].offset);  } printf("\n");
	fclose(file);
}

/* ********************************************************************************************** */
// Sets the position offsets for imu and schunk motor encoders
void Krang::set_position_offsets() {
	// Set imu offset
	imu->imu_offset = position[0].offset * M_PI /180;
	// Set waist offset
	double waist_offset_val[2];
	waist_offset_val[0] = position[1].offset * M_PI / 180.0;
	waist_offset_val[1] = -position[1].offset * M_PI / 180.0;
	waist->set_pos_offset(waist_offset_val);
	// Set torso offset
	double torso_offset[1] = { position[2].offset * M_PI / 180.0 };
	torso->set_pos_offset(torso_offset);
	// Set Arm offsets
	double llwa_offset[7], rlwa_offset[7];
	for(int i=0; i<7; i++) {
	  llwa_offset[i] = position[3+i].offset * M_PI / 180.0;
		rlwa_offset[i] = position[10+i].offset * M_PI / 180.0;
	}
	llwa->set_pos_offset(llwa_offset);
	rlwa->set_pos_offset(rlwa_offset);
	// Set amc offsets
	double amc_offset[2] = { -amc->get_pos(0)-imu->get_th(), -amc->get_pos(1)-imu->get_th() }; 
	amc->set_pos_offset(amc_offset);
}
/* ********************************************************************************************** */
// Reads the file containing values of positions sensed the last time before krang shut down. 
// If the file doesn't exist, assumes zero as default value. Saved in degrees.
void Krang::read_previous_positions_from_file() {
	// Open the binary file for reading
	FILE *file = fopen(lastPositionsPath, "rb");
	// If the file doesn't exist, create it and write zeros in it
	if(file==NULL) {
		//Give error message
		fprintf(stderr, "Could not open file containing last updated positions.\n");
		// Fill in zeros for the last positions
		for(int i=0;i<17;i++) {
			position[i].last = 0.0;
		}
		return;
	}
	// If the file exists read the values for the last positions before porogram exits
	for(int i=0;i<17;i++) {
		fread(&position[i].last, sizeof(double), 1, file);
	}
	fclose(file);
	
}

/* ********************************************************************************************** */
// Write position offsets to file
void Krang::write_position_offsets_to_file() {
	// Open file to write
	FILE *file = fopen(positionOffsetsPath,"wb");
	// Write the position offsets
	double val;
	for(int i=0; i<17; i++) {
		fwrite(&position[i].offset, sizeof(double), 1, file);
	}
	fclose(file);
}

/* ********************************************************************************************** */
// Write positions to file
void Krang::write_positions_to_file() {
	// Open file to write
	FILE *file = fopen(lastPositionsPath,"wb");
	// Write the position offsets
	for(int i=0; i<17; i++) {
		fwrite(&position[i].current, sizeof(double), 1, file);
	}
	fclose(file);
}

/* ********************************************************************************************** */
// Set all schunk module velocities to zero
void Krang::set_velocities_to_zero() {
	double waist_vel[2] = {0.0,0.0};
	double torso_vel[1] = {0.0};
	double arm_vel[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	waist->set_velocity(waist_vel);
	torso->set_velocity(torso_vel);
	rlwa->set_velocity(arm_vel);
	llwa->set_velocity(arm_vel);
	
}
/* ********************************************************************************************** */
Krang::~Krang() {

	// Halt the modules
	waist->halt();
	torso->halt();
	rlwa->halt();
	llwa->halt();

	populate_positions(); 
	write_positions_to_file();  
	
	delete js;
	delete imu;
	delete amc;
	delete waist;
	delete torso;
	delete rlwa;
	delete llwa;

	filter_kalman_destroy( kf );
	delete( kf );
}

/* ********************************************************************************************** */
void Krang::get_state(krang_state_t *X) {
	double  val[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	aa_fcpy(X->arm[KRANG_I_RIGHT].G.q, this->rlwa->motor.pos, 7);
	aa_fcpy(X->arm[KRANG_I_RIGHT].G.dq, this->rlwa->motor.vel, 7);
	aa_fcpy(X->arm[KRANG_I_LEFT].G.q, this->llwa->motor.pos, 7);
	aa_fcpy(X->arm[KRANG_I_LEFT].G.dq, this->llwa->motor.vel, 7);
}

/* ********************************************************************************************** */
void Krang::get_js(krang_js_t *ejs) {
	memcpy(ejs, &this->js->jsvals, sizeof(*ejs));
}

/* ********************************************************************************************** */
void Krang::update() {

	// Update all the devices connected
	js->update();
	imu->update();
	amc->update();
	waist->update();
	//torso->update();
	rlwa->update();
	llwa->update();
	imu->update();
	// Computes the arm CoM and sets the imu balancing angle 
	this->update_param();	   
}

/* ********************************************************************************************** */
void Krang::krang_dump_state( Krang *krang, double dt ) {
	printf("dt:\t%10.5f\t", dt);
	printf("q1_0:\t%10.5f\t", krang->state->q1_0);
	printf("dq1_0:\t%10.5f\t", krang->state->dq1_0);
	printf("q1_1:\t%10.5f\t", krang->state->q1_1);
	printf("dq1_t:\t%10.5f\t", krang->state->dq1_1);
	printf("q2:\t%10.5f\t", krang->state->q2);
	printf("dq2:\t%10.5f\t", krang->state->dq2);
	printf("q3:\t%10.5f\t", krang->state->q3);
	printf("dq3:\t%10.5f\t", krang->state->dq3);
	printf("rq1_0:\t%10.5f\t", krang->amc->get_pos(0));	 // to compare to q1_0
	printf("rdq1_0:\t%10.5f\t", krang->amc->get_vel(0));	 // to compare to dq1_0
	printf("rq1_t\t:%10.5f\t", krang->amc->get_pos(1));	 // to compare to q1_1
	printf("rdq1_t:\t%10.5f\t", krang->amc->get_vel(1));	 // to compare to dq1_1
	printf("th:\t%10.5f\t", krang->imu->get_th());
	printf("dth:\t%10.5f\t", krang->imu->get_dth());
	printf("w:\t%10.5f\t", krang->waist->get_pos(0));
	printf("dw:\t%10.5f\t", krang->waist->get_vel(0));
	printf("\n");
}


/* ********************************************************************************************** */
void Krang::update_and_filter() {

	// Look for updates on all the channels such as wheels, arms, joystick and etc.
	this->update();

	static bool debug = 0;
	static int counter = 0;

	// Set up the kalman filter with the raw data for imu, waist and amc
	// NOTE: The arm values are not filtered. Why not?
	kf->z[0] = imu->get_th();  // = q2
	kf->z[1] = imu->get_dth(); // = dq2
	kf->z[2] = amc->get_pos(0)+imu->get_th();  // = abs. L wheel pos.
	kf->z[3] = amc->get_vel(0)+imu->get_dth(); // = abs. L wheel vel.
	kf->z[4] = amc->get_pos(1)+imu->get_th();  // = abs. R wheel pos.
	kf->z[5] = amc->get_vel(1)+imu->get_dth(); // = abs. R wheel vel.
	kf->z[6] = waist->get_pos(0);
	kf->z[7] = waist->get_vel(0);

	// Call the kalman filter
	krang_kalman_filter();

	// Set the filtered values to the local fields
	state->q2		  = kf->x[0];
	state->dq2		 = kf->x[1];
	state->q1_0		= kf->x[2] - state->q2;
	state->dq1_0	   = kf->x[3] - state->dq2;
	state->q1_1		= kf->x[4] - state->q2;
	state->dq1_1	   = kf->x[5] - state->dq2;
	state->q3		  = kf->x[6];
	state->dq3		 = kf->x[7];

	// Useful debugging information to calibrate the kalman filter constants
	if(debug) {
		printf("%lf\t%lf\t%lf\t%lf\n", imu->get_th(), imu->get_dth(), kf->x[0], kf->x[1]);
		counter++;
		if(counter % 500 == 1) {
		  // printf("counter: %lu\n", counter);
			fflush(stdout);
		}
	}

	// Set the wheel position by taking the mean of the two wheel encoders 
	state->q1 = (state->q1_0 + state->q1_1)/2.0;
	state->dq1 = (state->dq1_0 + state->dq1_1)/2.0;
}

/* ********************************************************************************************** */
void Krang::krang_kalman_filter(/*double* state, double *raw_data*/) {

	// For now, assume fixed.
	double T = 2.04 * 1e-3;  // second

	static bool init_kf1 = false;
	if (!init_kf1) {
		memcpy(kf->x, kf->z, sizeof(double)*8);
		init_kf1 = true;
	}

	// Process matrix - fill every 9th value to 1 and every 18th starting from 8 to T.
	for(size_t i = 0; i < 64; i += 9)
		kf->A[i] = 1.0;
	for(size_t i = 8; i < 64; i += 18)
		kf->A[i] = T;

	// Process noise covariance matrix
	const double k1 = 2.0;
	const double k1b = 5.0;
	const double k2 = 10.0;
	const double k3 = 1.0;
	kf->R[0] = (T*T*T*T)*k1*(1.0/4.0);
	kf->R[1] = (T*T*T)*k1*(1.0/2.0);
	kf->R[8] = (T*T*T)*k1*(1.0/2.0);
	kf->R[9] = (T*T)*k1b;
	kf->R[18] = (T*T*T*T)*k2*(1.0/4.0);
	kf->R[19] = (T*T*T)*k2*(1.0/2.0);
	kf->R[26] = (T*T*T)*k2*(1.0/2.0);
	kf->R[27] = (T*T)*k2;
	kf->R[36] = (T*T*T*T)*k2*(1.0/4.0);
	kf->R[37] = (T*T*T)*k2*(1.0/2.0);
	kf->R[44] = (T*T*T)*k2*(1.0/2.0);
	kf->R[45] = (T*T)*k2;
	kf->R[54] = (T*T*T*T)*k3*(1.0/4.0);
	kf->R[55] = (T*T*T)*k3*(1.0/2.0);
	kf->R[62] = (T*T*T)*k3*(1.0/2.0);
	kf->R[63] = (T*T)*k3;

	// Measurement matrix - fill every 9th value to 1
	for(size_t i = 0; i < 64; i += 9)
		kf->C[i] = 1.0;

	// Measurement noise covariance matrix
	double imuCov = 1e-3;	//1e-3
	kf->Q[0] = imuCov;	// IMU
	kf->Q[9] = imuCov;

	kf->Q[18] = 0.0005; // AMC
	kf->Q[27] = 0.02;

	kf->Q[36] = 0.0005; // AMC
	kf->Q[45] = 0.02;

	kf->Q[54] = 0.05;   // Torso
	kf->Q[63] = 0.001;

	filter_kalman_predict( kf );
	filter_kalman_correct( kf );
}

/* ********************************************************************************************** */
void Krang::update_param() {

	// =========================================================
	// Compute the center of mass of each arm

	// Set the joint values to prepare for the call
	double qr[7], ql[7];
	double val[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	aa_fcpy(qr, this->rlwa->motor.pos, 7);
	aa_fcpy(ql, this->llwa->motor.pos, 7);

	// Compute the CoM by calling the schunkin library
	double rcm[3], lcm[3];
	double temp [3] = {0,0,0};
	double temp_ident[12] = {1,0,0, 0,1,0, 0,0,1, 0,0,0};
	schkin_lwa3_com(qr, temp_ident, temp, temp_ident, temp, .065,2.65, rcm);
	schkin_lwa3_com(ql, temp_ident, temp, temp_ident, temp, .065,2.65, lcm);
	static int cnt = 0;
	if(cnt++ % 200 == 201) {
		printf("%d: ql: %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f", cnt / 200, ql[0], ql[1],ql[2],ql[3],ql[4],ql[5],ql[6]);
		printf("\t\tqr: %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f,\n", qr[0], qr[1],qr[2],qr[3],qr[4],qr[5],qr[6]);
	}
	// =========================================================
	// Compute the imu balancing angle

	// x, z values
	// NOTE the center of mass locations in this section are weighted by the mass of the link. They
	// are NOT strictly locations.
	// TODO account for torso angle if it is not already done in rcm/lcm
	// cm_2 array length 2, 1st is com parallel to wheels, 2nd is com height above axle
  double cm_2[2] = { L2_m*L2_com_len*sin(state->q2), L2_m*L2_com_len*cos(state->q2) };
	// x_2 gives us the position of the waist center relative to wheel axle reference frame
	double x_2[2] = { L2_len*sin(state->q2), L2_len*cos(state->q2) };
	// cm_3 gives the position of the center of mass of the spine but assumes it to be at the end
	// (L3_len) assumed 4x4 Link is massless
	double sumAngle = state->q2 + state->q3;
	double cm_3[2] = { L3_m * ( x_2[0] + L3_len*sin(sumAngle) ),
					   L3_m * ( x_2[1] + L3_len*cos(sumAngle) ) };
	//x_3 gives us the position of the upper end of the spine
	double x_3[2] = { x_2[0] + L3_len*sin(sumAngle), x_2[1] + L3_len*cos(sumAngle) };
	//cm_r gives us the position of the center of mass of the right arm
	double cm_r[2] = { L4_m * (x_3[0]+rcm[2]),
					   L4_m * (x_3[1]+rcm[1]) };
	//cm_l gives us the position of the center of mass of the left arm
	double cm_l[2] = { L4_m * (x_3[0]-lcm[2]),
					   L4_m * (x_3[1]-lcm[1]) };
	// cm contains the sum of the weighted center of mass locations, divided by the total mass of the
	// components, giving center of mass of Krang
	double cm[2];
	for( size_t i = 0; i < 2; i ++ )
		cm[i] = (cm_2[i] + cm_3[i] + cm_r[i] + cm_l[i]) / (L2_m+L3_m+L4_m+L4_m);
	//Computes the angle from the origin on the axle to the center of mass
	double a = atan2( cm[0], cm[1] );
	// Calculates the difference between the measured imu angle and the angle to the center of mass
	double n_imu_ref = state->q2 - a;
	// Sets the difference as imu_balancing_angle
//	imu_balancing_angle = n_imu_ref;
	state->imu_ref = n_imu_ref;
}

