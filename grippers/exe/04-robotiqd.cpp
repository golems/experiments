/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file 04-daemon.cpp
 * @author Saul Reynolds-Haertle
 * @date June 26, 2013

 * @brief This demo communicates with the robotiq daemon to
 * demonstrate a number of the gripper's features and behaviors. It
 * runs the hands through several basic grasps to demonstrate
 * grasp-switching behavior, then demonstrates velocity control for
 * full grasps, then switches to independent mode to do the same
 * demonstrations for one finger at a time.
 *
 * This demonstration will not work unless the robotiq daemon is
 * running! Run it something like this:
 *   ach -C gripper-cmd
 *   ach -C gripper-state
 *   sudo robotiqd --bus=2 --module=11
 */



#include <somatic.h>
#include <ach.h>
#include <robotiqd.h>
#include <unistd.h>
#include <string.h>



int main(int argc, char* argv[]) {
    // open ach channel for talking to the daemon
    ach_channel_t robotiqd_cmd;
    ach_open(&robotiqd_cmd, "gripper-cmd", NULL);

    // our message struct
    robotiqd_achcommand_t msg;

    // First: move back to basic open position. In the GRASP modes,
    // all three of the settings are sent and paid attention to, so we
    // set them here.
    msg.mode = GRASP_BASIC;
    msg.grasping_pos = 0x00;    // all values are bytes
    msg.grasping_speed = 0xff;
    msg.grasping_force = 0xff;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Basic grasp\n");
    sleep(10);

    // Second: move to pinch open position. We reuse the same struct,
    // so no need to set the pos, speed, force again.
    msg.mode = GRASP_PINCH;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Pinch grasp\n");
    sleep(10);


    // Third: move to basic close position to demonstrate what happens
    // when you change position and mode at the same time.
    msg.mode = GRASP_BASIC;
    msg.grasping_pos = 0xff;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Basic grasp close\n");
    sleep(10);


    // Fourth: move straight to pinch close position to demonstrate
    // switching behavior.
    msg.mode = GRASP_PINCH;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Pinch grasp close\n");
    sleep(10);

    // Fifth: move as slowly as possible to a "halfway open" grasp
    msg.grasping_pos = 0x88;
    msg.grasping_speed = 0x00;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Half-open\n");
    sleep(10);

    // Sixth: for completeness, demonstrate the wide and scissor
    // grasps
    msg.mode = GRASP_WIDE;
    msg.grasping_pos = 0xff;    // reset these
    msg.grasping_speed = 0xff;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Wide grasp \n");
    sleep(10);
    msg.mode = GRASP_SCISSOR;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Scissor grasp\n");
    sleep(10);

    // Seventh: Back to basic mode, then switch to independent with a
    // roughly similar configuration. the grasing_ values are now
    // completely ignored. We also use this message to set all the
    // individual finger values so we don't have to worry about what's
    // in them. As such, we set the entire fing_mask to tell the
    // daemon to pay attention to every fing flag.
    msg.mode = INDEPENDENT;
    msg.fing_mask = 0xfff;
    msg.fing_a_pos = 0x00;
    msg.fing_b_pos = 0x00;
    msg.fing_c_pos = 0x00;
    msg.fing_scis_pos = 0x88;

    msg.fing_a_speed = 0xff;
    msg.fing_a_force = 0xff;
    msg.fing_b_speed = 0xff;
    msg.fing_b_force = 0xff;
    msg.fing_c_speed = 0xff;
    msg.fing_c_force = 0xff;
    msg.fing_scis_speed = 0xff;
    msg.fing_scis_force = 0xff;

    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Independent mode\n");
    sleep(10);

    // Eighth: show some independent-mode movements.
    msg.mode = INDEPENDENT;
    msg.fing_mask = FING_MASK_A_POS | FING_MASK_B_POS | FING_MASK_C_POS;
    msg.fing_a_pos = 0xff;
    msg.fing_b_pos = 0xaa;
    msg.fing_c_pos = 0x55;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Fingers to different positions\n");
    sleep(10);

    // Nine: open back up
    msg.mode = INDEPENDENT;
    msg.fing_mask = FING_MASK_A_POS | FING_MASK_B_POS | FING_MASK_C_POS;
    msg.fing_a_pos = 0x00;
    msg.fing_b_pos = 0x00;
    msg.fing_c_pos = 0x00;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Open back up\n");
    sleep(10);

    // Ten: close at different rates
    msg.mode = INDEPENDENT;
    msg.fing_mask =
        FING_MASK_A_POS | FING_MASK_B_POS | FING_MASK_C_POS |
        FING_MASK_A_SPEED | FING_MASK_B_SPEED | FING_MASK_C_SPEED;
    msg.fing_a_pos = 0xff;
    msg.fing_b_pos = 0xff;
    msg.fing_c_pos = 0xff;
    msg.fing_a_speed = 0xff;
    msg.fing_b_speed = 0xaa;
    msg.fing_c_speed = 0x55;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Close at different speeds\n");
    sleep(10);

    // Eleven: set speeds back to full and start opening at various times
    msg.mode = INDEPENDENT;
    msg.fing_mask = FING_MASK_A_POS | FING_MASK_A_SPEED;
    msg.fing_a_pos = 0x00;
    msg.fing_a_speed = 0xff;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Staged opening\n");
    usleep(750000);
    msg.fing_mask = FING_MASK_B_POS | FING_MASK_B_SPEED;
    msg.fing_b_pos = 0x00;
    msg.fing_b_speed = 0xff;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    usleep(750000);
    msg.fing_mask = FING_MASK_C_POS | FING_MASK_C_SPEED;
    msg.fing_c_pos = 0x00;
    msg.fing_c_speed = 0xff;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    sleep(10);

    // Twelve: switching from a GRASP to INDEPENDENT immediately moves
    // the hand to the last commanded independent-mode configuration,
    // but there's a catch: the grasping_pos and fing_a_pos values are
    // stored in the same register, so fing a won't move but the other
    // three dofs will.
    msg.mode = GRASP_BASIC;
    msg.grasping_pos = 0xff;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
    printf("Switching from GRASP to INDEPENDENT\n");
    sleep(4);
    msg.mode = INDEPENDENT;
    ach_put(&robotiqd_cmd, &msg, sizeof(msg));
}
