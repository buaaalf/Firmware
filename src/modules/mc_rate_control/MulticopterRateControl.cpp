/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

float
MulticopterRateControl::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_landed) {
		_gear_state_initialized = false;
	}

	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

	if (_manual_control_setpoint.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_setpoint.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		const bool manual_control_updated = _manual_control_setpoint_sub.update(&_manual_control_setpoint);

		// generate the rate setpoint from sticks?
		bool manual_rate_sp = false;

		if (_v_control_mode.flag_control_manual_enabled &&
		    !_v_control_mode.flag_control_altitude_enabled &&
		    !_v_control_mode.flag_control_velocity_enabled &&
		    !_v_control_mode.flag_control_position_enabled) {

			// landing gear controlled from stick inputs if we are in Manual/Stabilized mode
			//  limit landing gear update rate to 10 Hz
			if ((now - _landing_gear.timestamp) > 100_ms) {
				_landing_gear.landing_gear = get_landing_gear_state();
				_landing_gear.timestamp = hrt_absolute_time();
				_landing_gear_pub.publish(_landing_gear);
			}

			if (!_v_control_mode.flag_control_attitude_enabled) {
				manual_rate_sp = true;
			}

			// Check if we are in rattitude mode and the pilot is within the center threshold on pitch and roll
			//  if true then use published rate setpoint, otherwise generate from manual_control_setpoint (like acro)
			if (_v_control_mode.flag_control_rattitude_enabled) {
				manual_rate_sp =
					(fabsf(_manual_control_setpoint.y) > _param_mc_ratt_th.get()) ||
					(fabsf(_manual_control_setpoint.x) > _param_mc_ratt_th.get());
			}

		} else {
			_landing_gear_sub.update(&_landing_gear);
		}

		if (manual_rate_sp) {
			if (manual_control_updated) {

				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(_manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-_manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(_manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = _manual_control_setpoint.z;

				// publish rate setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = v_rates_sp.roll;
				_rates_sp(1) = v_rates_sp.pitch;
				_rates_sp(2) = v_rates_sp.yaw;
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from mixer feedback
			if (_motor_limits_sub.updated()) {
				multirotor_motor_limits_s motor_limits;

				if (_motor_limits_sub.copy(&motor_limits)) {
					MultirotorMixer::saturation_status saturation_status;
					saturation_status.value = motor_limits.saturation_status;

					_rate_control.setSaturationStatus(saturation_status);
				}
			}

			// run rate controller
			const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);
			
						//增加测试代码的地方：
			//输入：_manual_control_setpoint.x _manual_control_setpoint.y _manual_control_setpoint.r _thrust_sp = _manual_control_setpoint.z
			//输出：_thrust_sp ； att_control(0) att_control(1) att_control(2)
			/*
			%MC混控逻辑如下：
			%Channel 1 connects to the right (starboard) motor.
			%Channel 2 connects to the left (port) motor.
			
			%  电机1 = （油门-滚转（固定翼偏航））*0.5  *1000+1000 右电机  INDEX_ROLL  _manual_control_setpoint.y
			%  电机2 = （油门+滚转（固定翼偏航））*0.5  *1000+1000 左电机 
			%Channel 5 connects to the right (starboard) elevon. 
			%Channel 6 connects to the left (port) elevon.
			%  舵机1 = （偏航（固定翼滚转）- 俯仰）*0.75 *500 +1500
			%  舵机2 = （偏航（固定翼滚转）+ 俯仰）*0.75 *500 +1500
			RW =RW1;
			*/
			
			///////////////////////////////定义变量及初始化：静态变量，存储上次状态////////////////////////////////////////////////////////////////
			//存储模式状态:
			//定义时间变量
			static hrt_abstime inittime= hrt_absolute_time();
			static hrt_abstime Throttle_inittime = hrt_absolute_time();
			//使用事件标志进行.
			static uint8_t AutoPhaseFlag = 0;
			static bool  Throttle_Change_flag = 0;
			static float Last__manual_control_setpoint_z = 0;
			static bool  Last_TestcontrolMode = 0;
			static bool  TestcontrolMode = 0;

			
			///////////////////////////////////更新和维护手动/自动状态////////////////////////////////////////////////////////////////
			Last_TestcontrolMode = TestcontrolMode;
			
			if ( !manual_rate_sp ) 
			{
				TestcontrolMode = 1;
			}
			else
			{
				TestcontrolMode = 0;
			}
			
			
			///////////////////////////////////手动模式处理，自动相关变量处理/////////////////////////////////////////////////////////////
			//如果是手动:直连相应手动指令并更新自动相关状态标志
			if ( TestcontrolMode == 0 ) //&& manual_control_updated
			{
				_thrust_sp =_manual_control_setpoint.z;
				att_control(1) = _manual_control_setpoint.x；

				//自动相关局部变量复位：
				AutoPhaseFlag = 0;
				inittime= hrt_absolute_time();
				
				Last__manual_control_setpoint_z = 0；
				Throttle_Change_flag = 0;
				Throttle_inittime = hrt_absolute_time();
			}

			
			///////////////////////////////////////油门信号处理//////////////////////////////////////////////////////////
			//如果是自动则执行以下动作：
			if (controlMode ==1)
			{
				//油门指令的处置：把油门杆当成阶跃指令
				if ( Last__manual_control_setpoint_z <=0.3 && _manual_control_setpoint.z >= 0.7 && Throttle_Change_flag == 0)
				{
					//要持续一段时间再重置
					Throttle_Change_flag = 1;
					Throttle_inittime = hrt_absolute_time();
				}
				//5秒时间
				else if (Throttle_Change_flag == 1 && _manual_control_setpoint.z >= 0.7)
				{
					if (hrt_absolute_time()-Throttle_inittime) > 5000000
					{
						//重新初始化舵面，并重新初始化舵机自动化序列相关变量
						AutoPhaseFlag = 1;
						inittime= hrt_absolute_time();
						att_control(1) = 0；//0度 舵面归中
										
						//油门增加0.1：
						if (_thrust_sp <=0.9)
						{
							_thrust_sp = _thrust_sp + 0.1;
						}

						Throttle_Change_flag = 0;	//下次跳出
					}
					else
					{
						Throttle_Change_flag = 0;
					}
				}
				//更新油门状态
				Last__manual_control_setpoint_z = _manual_control_setpoint.z;
			}

			
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//如果 之前是手动,目前是自动:则初始化舵机自动序列相关变量:
			if ( controlMode ==1 && Last_controlMode ==0)
			{
				AutoPhaseFlag = 1
				inittime= hrt_absolute_time();
				
				att_control(1) = 0; 	//舵面归中
				_thrust_sp = 0; 		//油门置零
			}
			
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//如果是自动则执行以下动作：
			if (controlMode ==1)
			{
				Switch (AutoPhaseFlag)
				{
					Case 1:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = -0.8；//0.02 40度
							AutoPhaseFlag = 2;
							inittime= hrt_absolute_time();
						}
						break;
					case 2:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = -0.7；//0.02 35度
							AutoPhaseFlag = 3;
							inittime= hrt_absolute_time();
						}
						break;
					case 3:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = -0.6；//0.02 
							AutoPhaseFlag = 4;
							inittime= hrt_absolute_time();
						}
						break;

					case 4:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = -0.5；//0.02 
							AutoPhaseFlag = 5;
							inittime= hrt_absolute_time();
						}
						break;
					case 5:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = -0.4；//0.02 
							AutoPhaseFlag = 6;
							inittime= hrt_absolute_time();
						}
						break;
					case 6:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = -0.3；//0.02 35度
							AutoPhaseFlag = 7;
							inittime= hrt_absolute_time();
						}
						break;
					case 7:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = -0.2；//0.02 
							AutoPhaseFlag = 8;
							inittime= hrt_absolute_time();
						}
						break;

					case 8:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = -0.1；//0.02 
							AutoPhaseFlag = 9;
							inittime= hrt_absolute_time();
						}
						break;
					case 9:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0；//0.02 
							AutoPhaseFlag = 10;
							inittime= hrt_absolute_time();
						}
						break;
					case 10:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0.1；//0.02 
							AutoPhaseFlag = 11;
							inittime= hrt_absolute_time();
						}
						break;
					case 11:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0.2；//0.02 
							AutoPhaseFlag = 12;
							inittime= hrt_absolute_time();
						}
						break;
					case 12:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0.3；//0.02 
							AutoPhaseFlag = 13;
							inittime= hrt_absolute_time();
						}
						break;
					case 13:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0.4；//0.02 
							AutoPhaseFlag = 14;
							inittime= hrt_absolute_time();
						}
						break;
					case 14:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0.5；//0.02 
							AutoPhaseFlag = 15;
							inittime= hrt_absolute_time();
						}
						break;
					case 15:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0.6；//0.02 
							AutoPhaseFlag = 16;
							inittime= hrt_absolute_time();
						}
						break;
					case 16:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0.7；//0.02 
							AutoPhaseFlag = 17;
							inittime= hrt_absolute_time();
						}
						break;
					case 17:
						If (hrt_absolute_time() – inittime)>30000000
						{
							att_control(1) = 0.8；//0.02 
							AutoPhaseFlag = 1;
							inittime= hrt_absolute_time();
						}
						break;

					default:
						break;
				}
			}
			
			
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//其他两通道置零：
			att_control(0) = 0;
			att_control(2) = 0;

			// publish actuator controls
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = (float)_landing_gear.landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			// scale effort by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status)) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						actuators.control[i] *= _battery_status_scale;
					}
				}
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

	perf_end(_loop_perf);
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
