/**
 * @file imu.cpp
 * @author Kasemsit Teeyapan, Can Erdogan
 * @date Aug 15, 2010
 * @brief The interface to talk with the imu.
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include <amino.h>
#include <schkin.h>
#include "krang.h"
#include "krang-io.h"

/* ********************************************************************************************** */
Imu::Imu(const char* chan_name, double _imu_offset){

	// Open the IMU channel
	int r  = ach_open( &imu_chan, chan_name , NULL );
	aa_hard_assert(r == ACH_OK, "Ach failure %s on opening IMU channel (%s, line %d)\n",
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Set the imu offset
	imu_offset = _imu_offset;

	// Sleep a bit and get the initial readings
	usleep(10000);
	this->update();
}

/* ********************************************************************************************** */
Imu::~Imu() {
	ach_close(&imu_chan);
}

/* ********************************************************************************************** */
void Imu::update(){

	// Get the IMU message
	int r;
	struct timespec abstime = aa_tm_future( aa_tm_sec2timespec( 1.0 / 30.0 ));
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imu_chan, &abstime );

	// Assert that there were no errors in the call
	aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME, 
			"Ach wait failure %s on IMU data receive (%s, line %d)\n",
			 ach_result_to_string(static_cast<ach_status_t>(r)),  __FILE__, __LINE__);

	// If a message was received, extract the pitch and its rate
	if (r == ACH_OK) {

		// Prepare the ssdmu structure 
		ssdmu_sample_t imu_sample;
		imu_sample.x  = imu_msg->data[0];
		imu_sample.y  = imu_msg->data[1];
		imu_sample.z  = imu_msg->data[2];
		imu_sample.dP = imu_msg->data[3];
		imu_sample.dQ = imu_msg->data[4];
		imu_sample.dR = imu_msg->data[5];

		// Make the calls to extract the pitch and rate of extraction
		th  = ssdmu_pitch(&imu_sample);				 
		dth = ssdmu_d_pitch(&imu_sample);			

		// Translate the pitch to the robot frame
		th = th + imu_offset;

		// Free the unpacked message
		somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );
	}
}
