/**
 * @file Controllers.h
 * @author Can Erdogan
 * @date Jan 14, 2013
 * @brief The definitions for the various controllers for different modes of operation.
 */

#include "krang.h" 

/// The set of controllers
class Controllers {
public:

	/// Controls the arm movement
	static void arm (krang_cx_t *cx, double dt, double *UR, double *UL);
};
