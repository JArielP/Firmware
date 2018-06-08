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
 * 1 :
 * 2 :
 * 3 :
 * 4 :
 * 5 :
 * 6 :
 * 7 :
 * 8 :
 *
 * You can NOT add maneuvers while it is already executing some
 *
 * @group System Identification
 * @min 0
 * @max 1023
 * @bit 0 --
 * @bit 1 fixed pitch (track airspeed)
 * @bit 2 fixed pitch (track pitch)
 * @bit 3 fixed pitch (track pitch, then const)
 * @bit 4 ramp on pitch
 * @bit 5 fixed elevator maneuver
 * @bit 6 211 with elevator
 * @bit 7 211 with ailerons
 * @bit 8 211 with rudder
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
 * Maximal amount of iterations
 *
 * Defines how many iteratioions ar flown in this maneuver
 *
 * @group System Identification
 * @min 0
 * @max 50
 */
PARAM_DEFINE_INT32(SID_2_ITER_MAX, 10);

/**
 * Angle Start
 *
 * Defines the the angle at witch the maneuver should start
 *
 * @group System Identification
 * @unit deg
 * @min -50
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_2_ANG_STOP, 10);

/**
 * Angle Stop
 *
 * Defines the the angle at witch the maneuver should stop
 *
 * @group System Identification
 * @unit deg
 * @min -50
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_2_ANG_START, 10);

/**
 * Angle Step
 *
 * Defines the amplitude of the steps that are taken after each maneuver.
 * If it is negative or not is defined by if angle start > angle stop or not
 *
 * @group System Identification
 * @unit deg
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_2_ANG_STEP, 10);

/**
 * Time to fix elevator
 *
 * Defines the time after witch the elevator gets fixed
 *
 * @group System Identification
 * @unit s
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_2_T_E_FIX, 3);

/**
 * Time constant
 *
 * Defines the time constant for the 211 pitch maneuver
 *
 * @group System Identification
 * @unit deg
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_4_TIME_CONST, 1);

/**
 * Actuator Steps
 *
 * Defines the steps down per iteration for the 211 pitch maneuver
 *
 * @group System Identification
 * @unit
 * @min 0
 * @max 50
 */
PARAM_DEFINE_FLOAT(SID_4_STEP, 0.01);

/**
 * Maximal amount of iterations
 *
 * Defines how many iteratioions ar flown in the 211 pitch maneuver
 *
 * @group System Identification
 * @min 0
 * @max 50
 */
PARAM_DEFINE_INT32(SID_4_ITER_MAX, 10);

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