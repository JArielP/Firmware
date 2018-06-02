/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
/* TODO: check if app allready added in cmake lists for pixhawk
 * TODO: add functionality of flying several times in one maneuver with different altitudes.
 */

#include "sys_id.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int SysID::print_usage(const char *reason) {
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("sys_id", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
    PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int SysID::print_status() {
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module

    return 0;
}

int SysID::custom_command(int argc, char *argv[]) {
    /*
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    // additional custom commands can be handled like this:
    if (!strcmp(argv[0], "do-something")) {
        get_instance()->do_something();
        return 0;
    }
     */

    return print_usage("unknown command");
}


int SysID::task_spawn(int argc, char *argv[])       // TODO: make sure task has enough RAM!
{
    _task_id = px4_task_spawn_cmd("module",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2048,
                                  (px4_main_t) &run_trampoline,
                                  (char *const *) argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

SysID *SysID::instantiate(int argc, char *argv[]) {
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'p':
                example_param = (int) strtol(myoptarg, nullptr, 10);
                break;

            case 'f':
                example_flag = true;
                break;

            case '?':
                error_flag = true;
                break;

            default:
                PX4_WARN("unrecognized flag");
                error_flag = true;
                break;
        }
    }

    if (error_flag) {
        return nullptr;
    }

    SysID *instance = new SysID(example_param, example_flag);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

SysID::SysID(int example_param, bool example_flag)
        : ModuleParams(nullptr) {
}

void
SysID::vehicle_status_poll() {
    bool updated;

    orb_check(_vehicle_status_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
    }
}

void
SysID::actuator_poll() {
    bool updated;

    orb_check(_virtual_actuator_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(actuator_controls_virtual_sys_id), _virtual_actuator_sub, &_virtual_actuator);
    }
}


void
SysID::airspeed_poll() {
    bool updated;

    orb_check(_airspeed_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
    }
}

void
SysID::sys_id_poll() {
    bool updated;

    orb_check(_sys_id_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(system_identification), _sys_id_sub, &_sys_id);
    }
}

void
SysID::vehicle_local_pos_poll() {
    bool updated;

    orb_check(_vehicle_local_pos_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_pos_sub, &_vehicle_local_pos);
    }
}

void SysID::set_vehicle_status() {
    if (_vehicle_status_pub != nullptr) {
        orb_publish(_vehicle_status_id, _vehicle_status_pub, &_vehicle_status);

    } else if (_vehicle_status_id) {
        _vehicle_status_pub = orb_advertise(_vehicle_status_id, &_vehicle_status);
    }
}

void SysID::set_sys_id_topic() {
    _sys_id.timestamp = hrt_absolute_time();
    if (_sys_id_pub != nullptr) {
        orb_publish(_sys_id_id, _sys_id_pub, &_sys_id);

    } else if (_sys_id_id) {
        _sys_id_pub = orb_advertise(_sys_id_id, &_sys_id);
    }
}


void SysID::set_attitude(vehicle_attitude_setpoint_s &att_sp) {
    att_sp.timestamp = hrt_absolute_time();
    att_sp.pitch_body = math::radians(att_sp.pitch_body);
    att_sp.yaw_body = math::radians(att_sp.yaw_body);
    att_sp.roll_body = math::radians(att_sp.roll_body);


    Quatf q(Eulerf(att_sp.roll_body, att_sp.pitch_body, att_sp.yaw_body));
    q.copyTo(att_sp.q_d);
    att_sp.q_d_valid = true;

    if (_attitude_sp_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(_attitude_sp_id, _attitude_sp_pub, &att_sp);

    } else if (_attitude_sp_id) {
        /* advertise the attitude rates setpoint */
        _attitude_sp_pub = orb_advertise(_attitude_sp_id, &att_sp);
    }
}

void SysID::set_rates(actuator_controls_s &_actuator) {
    /* publish the actuator controls */
    if (_actuators_0_pub != nullptr) {
        orb_publish(_actuators_id, _actuators_0_pub, &_actuator);

    } else if (_actuators_id) {
        _actuators_0_pub = orb_advertise(_actuators_id, &_actuator);
    }
}


void SysID::control_pitch_aspeed(float airspeed_sp) {
    _att_sp.pitch_body = (_airspeed.indicated_airspeed_m_s - airspeed_sp) * airspeed_pitch_gain.get();
    // PX4_INFO("attitude setpoint pitch %0.3f",(double)_att_sp.pitch_body);
    // check if attitude setpoint is bigger than 5deg or smaller than 10deg:
    if (_att_sp.pitch_body < pitch_min.get()) {
        _att_sp.pitch_body = pitch_min.get();
    } else if (_att_sp.pitch_body > pitch_max.get()) {
        _att_sp.pitch_body = pitch_max.get();
    }
    // PX4_INFO("pitch setpoint is equal to: %.3f", (double)_att_sp.pitch_body);
}

void SysID::run() {
    // Example: run the loop synchronized to the sensor_combined topic publication
    int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _virtual_actuator_sub = orb_subscribe(ORB_ID(actuator_controls_virtual_sys_id));
    _sys_id_sub = orb_subscribe(ORB_ID(system_identification));
    _vehicle_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _airspeed_sub = orb_subscribe(ORB_ID(airspeed));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));


    _actuators_id = ORB_ID(actuator_controls_0);
    _vehicle_status_id = ORB_ID(vehicle_status);
    _attitude_sp_id = ORB_ID(vehicle_attitude_setpoint); // _attitude_sp_id = ORB_ID(fw_virtual_attitude_setpoint);
    _sys_id_id = ORB_ID(system_identification);

    /* rate limit control mode updates to 5Hz */
    // orb_set_interval(_vehicle_command_sub, 200);

    px4_pollfd_struct_t fds[1];
    fds[0].fd = sensor_combined_sub;
    fds[0].events = POLLIN;

    // initialize parameters
    int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    parameters_update(parameter_update_sub, true);

    // polls the topics:
    sys_id_poll();
    actuator_poll();
    vehicle_status_poll();
    vehicle_local_pos_poll();
    airspeed_poll();

    _vehicle_status.in_sys_id_maneuver = false;
    set_vehicle_status();

    bool get_new_maneuver = false;
    bool all_maneuvers_finished = false;
    bool maneuver_finished = true;
    bool sys_id_reset = false;

    int iteration = 1;

    // float actuator_pitch = 0.0f;

    /*
     * if status_changed, then the set_vehicle_status() function will be prompted at the end in order
     * to publish the new status
     */
    bool status_changed = false;
    set_sys_id_topic();

    while (!should_exit()) {
        // TODO: sysnchronize with an other topic.
        // wait for up to 1000ms for data
        int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

        if (pret == 0) {
            // Timeout: let the loop run anyway, don't do `continue` here

        } else if (pret < 0) {
            // this is undesirable but not much we can do
            PX4_ERR("poll error %d, %d", pret, errno);
            usleep(50000);
            continue;

        } else if (fds[0].revents & POLLIN) {

            struct sensor_combined_s sensor_combined;
            orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);

            // polls the topics:
            sys_id_poll();
            actuator_poll();
            vehicle_status_poll();
            vehicle_local_pos_poll();
            airspeed_poll();

            if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_SYSID) {
                if (sys_id_reset) {
                    _sys_id.mode = 0;
                    get_new_maneuver = false;
                    sys_id_reset = false;
                    all_maneuvers_finished = false;
                    maneuver_finished = true;
                }
                if (!all_maneuvers_finished) {
                    if (!_vehicle_status.in_sys_id_maneuver) {
                        /* evalueates the condition for a new maneuver
                         * if there is no more maneuver to fly, it will stay in the first loop
                         */
                        math::Vector<2> bearing(_vehicle_local_pos.vx, _vehicle_local_pos.vy);
                        bearing.normalize();
                        math::Vector<2> target_bearing(cosf(math::radians(direction.get())),
                                                       sinf(math::radians(direction.get())));

                        if (_vehicle_local_pos.z < -altitude_start.get() && target_bearing * bearing > 0.98f) {
                            if (maneuver_finished) {
                                get_new_maneuver = true;
                                // mavlink_log_critical(&_mavlink_log_pub, "[sys_id] getting new mode");
                                iteration = 1;
                            } else {
                                /*
                                 * initialize sys_id maneuver
                                 */
                                get_new_maneuver = false;
                                maneuver_finished = false;
                                iteration++;
                                _vehicle_status.in_sys_id_maneuver = true;
                                status_changed = true;
                                _sys_id.maneuver_valid = true;
                                _sys_id.timestamp_start_maneuver = hrt_absolute_time();
                                _att_sp.roll_body = 0.0f;
                                _att_sp.pitch_body = 0.0f;
                                _att_sp.yaw_body = 0.0f;
                                _att_sp.thrust = 0.0f;
                                set_attitude(_att_sp); // roll, pitch, yaw, thrust
                                mavlink_log_critical(&_mavlink_log_pub, "[sys_id] stay in same mode, iteration = %d",
                                                     iteration);
                            }
                        }

                    } else {
                        // TODO elaborate sys_id modes
                        switch (_sys_id.mode) {
                            case system_identification_s::MODE_NONE:
                                get_new_maneuver = true;
                                break;

                            case system_identification_s::MODE_FIXED_PITCH:
                                control_pitch_aspeed(airspeed_start.get() - (float) iteration * step_1.get());
                                set_attitude(_att_sp);
                                set_rates(_virtual_actuator);

                                /*
                                 * conditon for exiting the sys_id mode of constant pitch gliding
                                 */
                                if ((_virtual_actuator.control[actuator_controls_s::INDEX_PITCH] >
                                     actuator_pitch_treshold.get() ||
                                     _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] <
                                     -actuator_pitch_treshold.get()) && !maneuver_finished) {
                                    maneuver_finished = true;
                                    mavlink_log_critical(&_mavlink_log_pub,
                                                         "[sys_id] ex maneuver %d bcs acctuator_elev > %0.2f",
                                                         system_identification_s::MODE_FIXED_PITCH,
                                                         (double) actuator_pitch_treshold.get());
                                } else if (_att_sp.pitch_body >= pitch_max.get() && !maneuver_finished) {
                                    maneuver_finished = true;
                                    mavlink_log_critical(&_mavlink_log_pub,
                                                         "[sys_id] ex maneuver %d bcs ptich_body > %0.2f",
                                                         system_identification_s::MODE_FIXED_PITCH,
                                                         (double) pitch_max.get());
                                } else if (iteration == iter_max_1.get() && !maneuver_finished) {
                                    maneuver_finished = true;
                                    mavlink_log_critical(&_mavlink_log_pub,
                                                         "[sys_id] ex maneuver %d bcs iteration = %d",
                                                         system_identification_s::MODE_FIXED_PITCH,
                                                         iter_max_1.get());
                                }
                                break;

                            case system_identification_s::MODE_FIXED_ELEVATOR:
                                set_attitude(_att_sp);
                                set_rates(_virtual_actuator);
                                maneuver_finished = true;
                                break;

                            case system_identification_s::MODE_211_ROLL:
                                set_attitude(_att_sp);
                                set_rates(_virtual_actuator);
                                maneuver_finished = true;
                                break;

                            case system_identification_s::MODE_211_PITCH: {
                                set_attitude(_att_sp);
                                if (iteration == 1) {
                                    _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] = 0.0f;
                                } else {
                                    float start_time = (time.get() - time_const_4.get() * 4.0f) / 2.0f;
                                    if (hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) < start_time * 1000000.0f) {
                                        _virtual_actuator.timestamp = hrt_absolute_time();
                                        _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] = 0.0f;
                                    } else if (
                                            hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) >
                                            start_time * 1000000.0f &&
                                            hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) <
                                            (start_time + 2.0f * time_const_4.get()) * 1000000.0f) {      // 2
                                        _virtual_actuator.timestamp = hrt_absolute_time();
                                        _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] =
                                                0.01f + ((float) iteration * step_4.get());
                                    } else if (hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) >
                                               (start_time + 2.0f * time_const_4.get()) * 1000000.0f &&
                                               hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) <
                                               (start_time + 3.0f * time_const_4.get()) * 1000000.0f) {   // 1
                                        _virtual_actuator.timestamp = hrt_absolute_time();
                                        _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] =
                                                -0.01f - ((float) iteration * step_4.get());
                                    } else if (hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) >
                                               (start_time + 3.0f * time_const_4.get()) * 1000000.0f &&
                                               hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) <
                                               (start_time + 4.0f * time_const_4.get()) * 1000000.0f) {   // 1
                                        _virtual_actuator.timestamp = hrt_absolute_time();
                                        _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] =
                                                0.01f + ((float) iteration * step_4.get());
                                    } else if (hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) >
                                               (start_time + 4.0f * time_const_4.get()) * 1000000.0f) {
                                        _virtual_actuator.timestamp = hrt_absolute_time();
                                        _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] = 0.0f;
                                    }
                                }
                                _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] += trim_pitch.get();
                                set_rates(_virtual_actuator);

                                /*
                                 * conditon for exiting the sys_id mode of 211 pitch
                                 */
                                if ((_virtual_actuator.control[actuator_controls_s::INDEX_PITCH] >
                                     actuator_pitch_treshold.get() ||
                                     _virtual_actuator.control[actuator_controls_s::INDEX_PITCH] <
                                     -actuator_pitch_treshold.get()) && !maneuver_finished) {
                                    maneuver_finished = true;
                                    mavlink_log_critical(&_mavlink_log_pub,
                                                         "[sys_id] ex maneuver %d bcs acctuator_elev > %0.2f",
                                                         system_identification_s::MODE_211_PITCH,
                                                         (double) actuator_pitch_treshold.get());
                                } else if (_att_sp.pitch_body >= pitch_max.get() && !maneuver_finished) {
                                    maneuver_finished = true;
                                    mavlink_log_critical(&_mavlink_log_pub,
                                                         "[sys_id] ex maneuver %d bcs ptich_body > %0.2f",
                                                         system_identification_s::MODE_211_PITCH,
                                                         (double) pitch_max.get());
                                } else if (iteration == iter_max_4.get() && !maneuver_finished) {
                                    maneuver_finished = true;
                                    mavlink_log_critical(&_mavlink_log_pub,
                                                         "[sys_id] ex maneuver %d bcs iteration = %d",
                                                         system_identification_s::MODE_211_PITCH,
                                                         iter_max_4.get());
                                }
                                break;
                            }

                            case system_identification_s::MODE_211_YAW:
                                set_attitude(_att_sp);
                                set_rates(_virtual_actuator);
                                maneuver_finished = true;
                                break;

                            default:
                                break;
                        }
                        /*
                         * conditions to end the one maneuver
                         */
                        if (hrt_elapsed_time(&_sys_id.timestamp_start_maneuver) >
                            (uint) (time.get() * 1000000.0f)) { // time for maneuver: 20s
                            _vehicle_status.in_sys_id_maneuver = false; // resets the integrators of attitude controller!
                            status_changed = true;
                        } else if (-_vehicle_local_pos.z < altitude_stop.get()) { //TODO: check for body roll
                            _vehicle_status.in_sys_id_maneuver = false; // resets the integrators of attitude controller!
                            status_changed = true;
                        }
                    }

                    /* reads out the new maneuver
                     * stops the system identification process if there are no more maneuver to be done
                     */
                    uint8_t old_mode = _sys_id.mode;
                    if (get_new_maneuver) {
                        /* decide witch sys_id maneuver to run
                         * with a bitmask
                         */
                        if (_sys_id.mode == 0) {
                            _sys_id.mode = 1;
                        }
                        while (!((sys_id_modes.get() & _sys_id.mode * 2) == _sys_id.mode * 2) &&
                               _sys_id.mode < system_identification_s::MODE_MAX) {
                            _sys_id.mode *= 2;
                        }
                        if (_sys_id.mode == system_identification_s::MODE_MAX) {
                            _vehicle_status.in_sys_id_maneuver = false;
                            status_changed = true;
                            all_maneuvers_finished = true;
                        } else {
                            _sys_id.mode = _sys_id.mode * 2;
                        }
                    }
                    if (old_mode != _sys_id.mode) {
                        /*
                         * initialize sys_id mode
                         */
                        mavlink_log_critical(&_mavlink_log_pub, "[sys_id] starting system identification mode %d",
                                             _sys_id.mode);
                        get_new_maneuver = false;
                        maneuver_finished = false;
                        _vehicle_status.in_sys_id_maneuver = true;
                        status_changed = true;
                        _sys_id.maneuver_valid = true;
                        _sys_id.timestamp_start_maneuver = hrt_absolute_time();
                        _att_sp.roll_body = 0.0f;
                        _att_sp.pitch_body = 0.0f;
                        _att_sp.yaw_body = 0.0f;
                        _att_sp.thrust = 0.0f;
                        set_attitude(_att_sp);
                        // reset_attitude_integrators();
                    }
                    if (all_maneuvers_finished) {
                        mavlink_log_critical(&_mavlink_log_pub,
                                             "[sys_id] system identification maneuvers finished");
                        _vehicle_status.in_sys_id_maneuver = false;
                        status_changed = true;
                    }
                    if (status_changed) {
                        set_vehicle_status();
                    }
                }
            } else {
                sys_id_reset = true;
                if (_vehicle_status.in_sys_id_maneuver) {
                    _vehicle_status.in_sys_id_maneuver = false;
                    set_vehicle_status();
                }
            }
            set_sys_id_topic();
        }

        parameters_update(parameter_update_sub);
    }

    orb_unsubscribe(sensor_combined_sub);
    orb_unsubscribe(parameter_update_sub);
    orb_unsubscribe(_vehicle_status_sub);
    orb_unsubscribe(_virtual_actuator_sub);
    orb_unsubscribe(_sys_id_sub);
    orb_unsubscribe(_vehicle_local_pos_sub);
    orb_unsubscribe(_airspeed_sub);
}


void SysID::parameters_update(int parameter_update_sub,
                              bool force) // TODO: search for this to get an example how to handle parameters
{
    bool updated;
    struct parameter_update_s param_upd;

    orb_check(parameter_update_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
    }

    if (force || updated) {
        updateParams();
    }
}


int sys_id_main(int argc, char *argv[]) {
    return SysID::main(argc, argv);
}
