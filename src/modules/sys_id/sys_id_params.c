/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sys_id_params.c
 *
 * Parameters of system identificaton
 *
 * @author Jonas Peter <peterjo@ethz.ch>
 */

#include <systemlib/param/param.h>

/**
 * System Identification modes
 *
 * Set bits in the following positions to enable:
 * 0 : --
 * 1 : Set to true for fixed pitch maneuver
 * 2 : Set to true for fixed elevator maneuver
 * 3 : Set to true for 211 with elevator
 * 4 : Set to true for 211 with ailerons
 * 5 : Set to true for 211 with rudder
 *
 * You can NOT add maneuvers while it is already executing some
 *
 * @group System Identification
 * @min 0
 * @max 63
 * @bit 0 --
 * @bit 1 fixed pitch maneuver
 * @bit 2 fixed elevator maneuver
 * @bit 3 211 with elevator
 * @bit 4 211 with ailerons
 * @bit 5 211 with rudder
 */
PARAM_DEFINE_INT32(SID_MODES, 3);

/**
 * direction for maneuver
 *
 * sets the direction in witch the maneuver will be executed
 *
 * @group System Identification
 * @min 0
 * @max 360
 * @unit deg
 */
PARAM_DEFINE_FLOAT(SID_DIR, 0);

/**
 * elevator treshold
 *
 * If the elevator exeeds this treshold in a maneuver, the system this one maneuver will be finished and the system
 * proceeds to the next one
 *
 * @group System Identification
 * @min -1
 * @max 1
 */
PARAM_DEFINE_FLOAT(SID_ELEV_TH, 0.5);

/**
 * Meanuever Time
 *
 * Defines the time for witch the system will execute one maneuver.
 *
 * @group System Identification
 * @unit sec
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_TIME, 20);

/**
 * pitch airspeed gain
 *
 * Sets the pitch gain for tracking the airspeed with the pitch in a "fixed pitch" maneuver
 *
 * @group System Identification
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_1_A_PP, 1);

/**
 * Airspeed Start
 *
 * Defines the airspeed for the fixed pitch maneuver whitch it tries to track
 *
 * @group System Identification
 * @unit m/s
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_1_ASP_SART, 15);

/**
 * Airspeed Steps
 *
 * Defines the steps down for the fixed pitch maneuver
 *
 * @group System Identification
 * @unit m/s
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_1_ASP_STEP, 1);

/**
 * Maximal Pitch SP
 *
 * Defines the maximal pitch setpoint that can be set in fixed pitch mode
 *
 * @group System Identification
 * @unit deg
 * @min -30
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_1_PITCH_MAX, 5);

/**
 * Minimal Pitch SP
 *
 * Defines the minimal pitch setpoint that can be set in fixed pitch mode
 *
 * @group System Identification
 * @unit deg
 * @min -30
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_1_PITCH_MIN, -20);

/**
 * Maximal Angel Of Attack
 *
 * Defines the angle of attack, when passing, the maneuver will be ended in the fixed pitch maneuver
 *
 * @group System Identification
 * @unit deg
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_1_AOA_MAX, 10);

/**
 * Maximal amount of iterations
 *
 * Defines how many iteratioions ar flown in this maneuver
 *
 * @group System Identification
 * @min 0
 * @max 50
 */
PARAM_DEFINE_INT32(SID_1_ITER_MAX, 10);

/**
 * Altitude Stop
 *
 * @group System Identification
 * @unit m
 * @min 5
 * @max 200
 */
PARAM_DEFINE_FLOAT(SID_ALT_START, 80);

/**
 * Altitude Start
 *
 * @group System Identification
 * @unit m
 * @min 5
 * @max 200
 */
PARAM_DEFINE_FLOAT(SID_ALT_STOP, 20);