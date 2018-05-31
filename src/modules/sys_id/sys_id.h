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

#pragma once

#include <px4_module.h>
#include <px4_module_params.h>
#include <geo/geo.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/system_identification.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/airspeed.h>

using matrix::Eulerf;
using matrix::Quatf;

extern "C" __EXPORT int sys_id_main(int argc, char *argv[]);


class SID_TIME;

class SysID : public ModuleBase<SysID>, public ModuleParams
{
public:
    SysID(int example_param, bool example_flag);

	virtual ~SysID() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SysID *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	void set_vehicle_status();
	void set_rates(actuator_controls_s &_actuator);
	void set_attitude(vehicle_attitude_setpoint_s &_att_sp);
	void set_sys_id_topic();

	int _vehicle_status_sub{-1};
    int _virtual_actuator_sub{-1};
	int _sys_id_sub{-1};
    int _vehicle_local_pos_sub{-1};
    int _airspeed_sub{-1};
    int _params_sub{-1};

	vehicle_status_s _vehicle_status {};
    actuator_controls_s _virtual_actuator {};
    system_identification_s _sys_id {};
    vehicle_local_position_s _vehicle_local_pos {};
    airspeed_s _airspeed{};
    vehicle_attitude_setpoint_s _att_sp{};

	orb_advert_t	_actuators_0_pub{nullptr};		/**< actuator control group 0 setpoint */
	orb_id_t 		_actuators_id{nullptr};	// pointer to correct actuator controls0 uORB metadata structure

	//TODO: change description
	orb_advert_t	_vehicle_status_pub{nullptr};		/**< actuator control group 0 setpoint */
	orb_id_t 		_vehicle_status_id{nullptr};	// pointer to correct actuator controls0 uORB metadata structure

	//TODO: change description
	orb_advert_t	_attitude_sp_pub{nullptr};		/**< actuator control group 0 setpoint */
	orb_id_t 		_attitude_sp_id{nullptr};	// pointer to correct actuator controls0 uORB metadata structure

    //TODO: change description
    orb_advert_t	_sys_id_pub{nullptr};		/**< actuator control group 0 setpoint */
    orb_id_t 		_sys_id_id{nullptr};	// pointer to correct actuator controls0 uORB metadata structure

	orb_advert_t	_mavlink_log_pub{nullptr};		/**< actuator control group 0 setpoint */

	void vehicle_status_poll();
	void actuator_poll();
	void sys_id_poll();
    void vehicle_local_pos_poll();
    void airspeed_poll();

    void control_pitch_aspeed(float airspeed_sp);

    /**
     * Check for parameter changes and update them if needed.
     * @param parameter_update_sub uorb subscription to parameter_update
     * @param force for a parameter update
     */
    void parameters_update(int parameter_update_sub, bool force = false);

	DEFINE_PARAMETERS(
            (ParamInt<px4::params::SID_MODES>) sys_id_modes,                  /**< example parameter */
            (ParamFloat<px4::params::SID_DIR>) direction,                     /**< example parameter */
            (ParamFloat<px4::params::SID_1_A_PP>) airspeed_pitch_gain,        /**< example parameter */
            (ParamFloat<px4::params::SID_ELEV_TH>) actuator_pitch_treshold,   /**< example parameter */
            (ParamFloat<px4::params::SID_TIME>) time,                         /**< example parameter */
            (ParamFloat<px4::params::SID_1_ASP_SART>) airspeed_start,         /**< example parameter */
            (ParamFloat<px4::params::SID_1_ASP_STEP>) step,                   /**< example parameter */
            (ParamFloat<px4::params::SID_ALT_START>) altitude_start,          /**< example parameter */
            (ParamFloat<px4::params::SID_ALT_STOP>) altitude_stop,            /**< example parameter */
			(ParamFloat<px4::params::SID_1_PITCH_MAX>) pitch_max,             /**< example parameter */
			(ParamFloat<px4::params::SID_1_PITCH_MIN>) pitch_min,             /**< example parameter */
			(ParamFloat<px4::params::SID_1_AOA_MAX>) aoa_max,                 /**< example parameter */
            (ParamInt<px4::params::SID_1_ITER_MAX>) iter_max                /**< example parameter */
	)
};

