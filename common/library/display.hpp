/**
 * @file display.hpp
 * @author Saul Reynolds-Haertle
 * @date July 29, 2013
 * @brief Common code for displaying things with curses.
 */

#pragma once

#include <ncurses.h>
#include "util.h"

namespace Krang {

    // some configuration and utility variables
    // TODO: move the definitions of these to a more useful place
	extern int curses_display_precision;
	extern int curses_display_row;
	
	// functions for starting up and shutting down curses cleanly
	void init_curses();
	void destroy_curses();

	// functions for nicely formatted and aligned debug output
	void curses_display_vector(const VectorXd& v, const char* label = "", int column = 0, int color = COLOR_WHITE);
	void curses_display_matrix(const MatrixXd& m, const char* label = "", int column = 0, int color = COLOR_WHITE);
}
