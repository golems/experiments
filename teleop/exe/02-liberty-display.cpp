/**
 * @file 02-libery-display.c
 * @author Saul Reynolds-Haertle
 * @date 08 July 2013
 *
 * @brief This demo communicates with the liberty daemon to get the
 * locations of the liberty sensors, and then dumps them to the
 * terminal in a readable manner.
 */

#include <somatic.h>
#include <ach.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <ncurses.h>
#include <algorithm>

#define BUF_SIZE 1024

// we have to do this because the people that wrote the liberty daemon
// don't have the common courtesy to put up an include with their
// struct definition.
typedef struct liberty_data {
    float sensor[8][7];         // 8 sensors, 7 values per sensor per tick (3 translations plus a quat)
} liberty_data_t;

int main(int argc, char* argv[]) {
    // set up some variables
    int r;                      // return vals
    char* buf[BUF_SIZE];        // buffer for getting from ach
    size_t received_size;       // size we get from ach
    timespec timeout;           // timeout for our ach_wait
    liberty_data_t libdata;     // struct to hold liberty data

    // set up curses
    initscr();
    int maxrow;
    int maxcol;
    getmaxyx(stdscr, maxrow, maxcol);

    float xscale = (float)maxcol / 2.0;
    float yscale = (float)maxrow / 2.0;

    int last_r;
    int last_c;

    // set up liberty
    ach_channel_t liberty_chan; // ach channel that the liberty daemon publishes on
    r = ach_open(&liberty_chan, "liberty", NULL);
    aa_hard_assert(r == ACH_OK, "Ach failure %s on opening liberty channel (%s, line %d)\n",
                   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
    
    while(1) {
        clock_gettime(ACH_DEFAULT_CLOCK, &timeout);
        timeout.tv_sec += 1; // timeout is one second in the future
        r = ach_get(&liberty_chan, buf, size_t(BUF_SIZE), &received_size, &timeout, ACH_O_WAIT);
        if (r != ACH_OK && r != ACH_TIMEOUT) {
            // std::cout << "ach error: " << ach_result_to_string(static_cast<ach_status_t>(r)) << std::endl;
            continue;
        }
        
        memcpy(&libdata, buf, received_size); // copy data into struct
        // 0 is z
        // 1 is y
        // 2 is x
        // 3-6 are a quaternion
        
        int c = (int)(xscale * libdata.sensor[0][0]); // calculate where on the screen to display the x and y values
        int r = (int)(yscale * libdata.sensor[0][1]);
        
        mvprintw(last_r, last_c, " "); // erase last mark
        mvprintw(r, c, "X");  // put up a new mark
        last_r = r; // rememver where our mark is for next loop
        last_c = c;
        
        refresh(); // flip what we just renderd onto the screen
    }
}
