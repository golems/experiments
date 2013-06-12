/**
 * @file helpers.cpp
 * @author Can Erdogan
 * @date June 12, 2013
 * @brief This file contains some helper functions such as reading force/torque data if available.
 */

#include "helpers.h"

/* ********************************************************************************************* */
bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data) {

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &ft_chan, &numBytes, &abstimeout, 
		ACH_O_LAST, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return false;

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return false;
	if(msg->meta->type != SOMATIC__MSG_TYPE__FORCE_MOMENT) return false;

	// Read the force-torque message and write it into the vector
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(daemon_cx.pballoc), 
		numBytes, buffer);
	for(size_t i = 0; i < 3; i++) data(i) = ftMessage->force->data[i]; 
	for(size_t i = 0; i < 3; i++) data(i+3) = ftMessage->moment->data[i]; 
	return true;
}
/* ********************************************************************************************* */
