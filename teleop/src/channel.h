/*
 * channel.h
 *
 *  Created on: Jul 14, 2013
 *      Author: jscholz
 */

#ifndef CHANNEL_H_
#define CHANNEL_H_

#include <amino.h>
#include <ach.h>

void ach_init(ach_channel_t* chan, char* chan_name, uint8_t *achbuf, size_t n_achbuf) {
	// Set up the buffer
	achbuf = AA_NEW_AR(uint8_t,  n_achbuf );

	// Open the given channel
	int r = ach_open( chan, chan_name, NULL );
	aa_hard_assert( ACH_OK == r, "Couldn't open channel %s\n", chan_name );
	r = ach_flush( chan );
	aa_hard_assert( ACH_OK == r, "Couldn't flush channel\n");
}


#endif /* CHANNEL_H_ */
