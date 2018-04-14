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
 *
 */

#include "sys_id.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int SysID::print_usage(const char *reason)
{
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

int SysID::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int SysID::custom_command(int argc, char *argv[])
{
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
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

SysID *SysID::instantiate(int argc, char *argv[])
{
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
			example_param = (int)strtol(myoptarg, nullptr, 10);
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
	: ModuleParams(nullptr)
{
}

void
SysID::vehicle_command_poll()
{
	bool updated;

	orb_check(_vehicle_command_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &_vehicle_command);
		// handle_command();
		// PX4_INFO("vehicle_command: ");
	}
}

void
SysID::vehicle_status_poll(){
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		// PX4_INFO("veicle_status.nav_state is: %.3f", (double)_vehicle_status.nav_state);
	}
}

void
SysID::actuator_poll()
{
	bool updated;

	orb_check(_virtual_actuator_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_fw), _virtual_actuator_sub, &_virtual_actuator);
	}
}

void SysID::set_state(bool in_maneuver) {
    if (_vehicle_status.in_sys_id_maneuver != in_maneuver) {
        if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_SYSID) {
            _vehicle_status.in_sys_id_maneuver = in_maneuver;
        } else {
            _vehicle_status.in_sys_id_maneuver = false;
        }
        // publishes the result:
        if (_vehicle_status_pub != nullptr) {
            orb_publish(_vehicle_status_id, _vehicle_status_pub, &_vehicle_status);

        } else if (_vehicle_status_id) {
            _vehicle_status_pub = orb_advertise(_vehicle_status_id, &_vehicle_status);
        }
    }
}

void SysID::set_attitude(float roll, float pitch, float yaw, float thrust) {
	// TODO: check if at given time this function is the only one who publishes the attitudes (should be OK)
    vehicle_attitude_setpoint_s _att_sp;
    _att_sp.timestamp = hrt_absolute_time();
    _att_sp.roll_body = roll;
    _att_sp.pitch_body = pitch;
    _att_sp.yaw_body = yaw;
    _att_sp.thrust = thrust;

    Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
    q.copyTo(_att_sp.q_d);
    _att_sp.q_d_valid = true;

    if (_attitude_sp_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(_attitude_sp_id, _attitude_sp_pub, &_att_sp);

    } else if (_attitude_sp_id) {
        /* advertise the attitude rates setpoint */
        _attitude_sp_pub = orb_advertise(_attitude_sp_id, &_att_sp);
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

void SysID::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    _vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_virtual_actuator_sub = orb_subscribe(ORB_ID(actuator_controls_virtual_fw));

	_actuators_id = ORB_ID(actuator_controls_0);
	_vehicle_status_id = ORB_ID(vehicle_status);
	_attitude_sp_id = ORB_ID(vehicle_attitude_setpoint);

    /* rate limit control mode updates to 5Hz */
    orb_set_interval(_vehicle_command_sub, 200);

	px4_pollfd_struct_t fds[1];
    fds[0].fd = sensor_combined_sub;
    fds[0].events = POLLIN;

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub, true);

	while (!should_exit()) {

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
            // TODO: poll sys_id topic
            // sys_id_poll();
			vehicle_command_poll();
			actuator_poll();
            vehicle_status_poll();

			/* sets the state according to if vehicle is in sys_id maneuver or preparing for it
			 * if not in "in_sys_id_maneuver", TODO: the mission code should run, with a mission set here.
			 * publishes to the vehicle_state topic, into in_sys_id_maneuver
			 * TODO: build logic around it!
			 */
            set_state(true);

            if (_vehicle_status.in_sys_id_maneuver) {
				/* sets the attitude controls acording to the desired controls only if "in_sys_id_maneuver"
				 * TODO: build logic around it:
				 **/
				set_attitude(0.0f, 0.0f, 0.0f, 0.0f); // roll, pitch, yaw, thrust
				/* sets the rates (actuator output) according to the desired rates
				 * TODO: build logic around it:
				 *      - if "in_sys_id_maneuver", the rates should be handled according to the sys_id maneuver
				 *      - else, the rates must not be published here, they are published in the attitude controller!!
				 */
                _virtual_actuator.timestamp = hrt_absolute_time(); // TODO: only put new timestamp if changed
                set_rates(_virtual_actuator);
			 }
		}

		parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(sensor_combined_sub);
	orb_unsubscribe(parameter_update_sub);
	orb_unsubscribe(_vehicle_command_sub);
	orb_unsubscribe(_vehicle_status_sub);
	orb_unsubscribe(_virtual_actuator_sub);
}



void SysID::parameters_update(int parameter_update_sub, bool force) // TODO: search for this to get an example how to handle parameters
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


int sys_id_main(int argc, char *argv[])
{
	return SysID::main(argc, argv);
}
