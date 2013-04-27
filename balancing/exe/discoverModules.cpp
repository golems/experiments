/**
 * @file discoverModules.cpp
 * @author Can Erdogan, Munzir Zafar
 * @date Jan 15, 2013
 * @brief This program attempts to open channels 1 to 16 and connect to the modules 1 to 31 at 
 * each channel. The output is the set of modules that could be connected and their respective
 * channels.
 */

#include <ntcan.h>
#include <ntcanopen.h>
#include <stdio.h>

// Set the begin and end ranges for channel and module numbers
static const size_t chan_begin = 0;
static const size_t chan_end = 16;
static const size_t module_begin = 0;
static const size_t module_end = 32;

/// Print the return message
const char *canResultString( int i ) { 
    NTCAN_RESULT ntr = i;
    switch( ntr ) { 
    case NTCAN_SUCCESS: return "SUCCESS";
    case NTCAN_RX_TIMEOUT: return "RX_TIMEOUT";
    case NTCAN_TX_TIMEOUT: return "TX_TIMEOUT";
    case NTCAN_TX_ERROR: return "TX_ERROR";
    case NTCAN_CONTR_OFF_BUS: return "CONTR_OFF_BUS";
    case NTCAN_CONTR_BUSY: return "CONTR_BUSY";
    case NTCAN_CONTR_WARN: return "CONTR_WARN";
    case NTCAN_NO_ID_ENABLED: return "NO_ID_ENABLED";
    case NTCAN_ID_ALREADY_ENABLED: return "ID_ALREADY_ENABLED";
    case NTCAN_INVALID_FIRMWARE: return "INVALID_FIRMWARE";
    case NTCAN_MESSAGE_LOST: return "MESSAGE_LOST";
    case NTCAN_INVALID_HARDWARE: return "INVALID_HARDWARE";
    case NTCAN_PENDING_WRITE: return "PENDING_WRITE";
    case NTCAN_PENDING_READ: return "PENDING_READ";
    case NTCAN_INVALID_DRIVER: return "INVALID_DRIVER";
    case NTCAN_SOCK_CONN_TIMEOUT: return "SOCK_CONN_TIMEOUT";
    case NTCAN_SOCK_CMD_TIMEOUT: return "SOCK_CMD_TIMEOUT";
    case NTCAN_SOCK_HOST_NOT_FOUND: return "SOCK_HOST_NOT_FOUND";
    case NTCAN_INVALID_PARAMETER: return "INVALID_PARAMETER";
    case NTCAN_INVALID_HANDLE: return "INVALID_HANDLE";
    case NTCAN_NET_NOT_FOUND: return "NET_NOT_FOUND";
    case NTCAN_INSUFFICIENT_RESOURCES: return "INSUFFICIENT_RESOURCES";
    case NTCAN_OPERATION_ABORTED: return "OPERATION_ABORTED";
    case NTCAN_WRONG_DEVICE_STATE: return "WRONG_DEVICE_STATE";
    case NTCAN_HANDLE_FORCED_CLOSE: return "HANDLE_FORCED_CLOSE";
    case NTCAN_NOT_IMPLEMENTED: return "NOT_IMPLEMENTED";
    case NTCAN_NOT_SUPPORTED: return "NOT_SUPPORTED";
    case NTCAN_CONTR_ERR_PASSIVE: return "CONTR_ERR_PASSIVE";
    default: return "unknown";
    }   
}

/// The main thread
int main () {

	// Attempt to open the can lines
	for(size_t chan_idx = chan_begin; chan_idx < chan_end; chan_idx++) {
		NTCAN_HANDLE handle;
		NTCAN_RESULT result = canOpen(chan_idx, 0, 8, 8, 100, 100, &handle); 

		// Attempt to bind to all the modules if opened correctly
		if(result == NTCAN_SUCCESS) {
			printf("Opened channel %d\n", chan_idx);
		} 
	
		// Print the error message
		else {
			printf("Error on attempt %d: '%s'\n", chan_idx, canResultString(result));
		}

	}
}
