/**
 * @file display.cpp
 * @author Saul Reynolds-Haertle
 * @date July 29, 2013
 * @brief Common code for displaying things with curses.
 */

#include "display.hpp"


namespace Krang {

	/* ############################################################################################## */
	/// Set up a curses display and some commonly used color pairs.
	void init_curses() {
		// tell people to do curses
		doing_curses = true;
		
		// general curses stuff
		initscr();
		clear();
		noecho();                   // do not echo input to the screen
		cbreak();                   // do not buffer by line (receive characters immediately)
		timeout(0);                 // non-blocking getch

		// color
		start_color();              // start printing in color
		init_pair(COLOR_RED, COLOR_RED, COLOR_BLACK);
		init_pair(COLOR_YELLOW, COLOR_YELLOW, COLOR_BLACK);
		init_pair(COLOR_GREEN, COLOR_GREEN, COLOR_BLACK);
		init_pair(COLOR_WHITE, COLOR_WHITE, COLOR_BLACK);
		init_pair(COLOR_RED_BACKGROUND, COLOR_WHITE, COLOR_RED);
		init_pair(COLOR_YELLOW_BACKGROUND, COLOR_BLACK, COLOR_YELLOW);
		init_pair(COLOR_GREEN_BACKGROUND, COLOR_BLACK, COLOR_GREEN);
		init_pair(COLOR_WHITE_BACKGROUND, COLOR_BLACK, COLOR_WHITE);
		
	}

	/* ############################################################################################## */
	/// Close down and clean up after curses
	void destroy_curses() {
		// tell people to stop doing curses
		doing_curses = true;
		
		// and clear out the curses configurations
		clrtoeol();
		refresh();
		endwin();
	}

	/* ############################################################################################## */
	/// Clean and easy function for displaying a vector in curses. This uses a global variable for
	/// keeping track of which rows have been printed to, so all you need to do to use this is make
	/// sure you set curses_display_row to something sensible at the beginning of every iteration
	/// of your main loop.
	void curses_display_vector(const VectorXd& v, const char* label, int column, int color) {
		attron(COLOR_PAIR(color));
		mvprintw(curses_display_row, 1, label);
		for(int i = 0; i < v.size(); i++)
			mvprintw(curses_display_row,
			         30 + ((12 * column) + i) * curses_display_precision,
			         "%.8lf", v[i]);
		curses_display_row++;
		attroff(COLOR_PAIR(color));
	}

	/* ############################################################################################## */
	/// Clean and easy function for displaying a matrix in curses. This uses a global variable for
	/// keeping track of which rows have been printed to, so all you need to do to use this is make
	/// sure you set curses_display_row to something sensible at the beginning of every iteration
	/// of your main loop.
	void curses_display_matrix(const MatrixXd& m, const char* label, int column, int color) {
		attron(COLOR_PAIR(color));
		mvprintw(curses_display_row, 1, label);
		for(int mrow = 0; mrow < m.rows(); mrow++) {
			for(int mcol = 0; mcol < m.cols(); mcol++)
				mvprintw(curses_display_row + mrow,
				         30 + ((12 * column) + mcol) * curses_display_precision,
				         "%.8lf", m(mrow,mcol));
			curses_display_row++;
		}
		attroff(COLOR_PAIR(color));
	}
}
