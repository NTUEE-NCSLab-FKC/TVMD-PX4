/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

/**
 * @file SquareTrajectory.cpp
 *
 * Fly a 1m x 1m square at 1m altitude with yaw rotation at corners.
 * Usage: square_trajectory start
 *
 * Sequence:
 *   1. Send setpoints for 5s (required before switching to offboard)
 *   2. Switch to OFFBOARD mode
 *   3. Arm
 *   4. Fly square trajectory with yaw rotation at each corner
 *   5. Land
 */

#include "SquareTrajectory.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

SquareTrajectory::SquareTrajectory() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool SquareTrajectory::init()
{
	_state = State::SEND_SETPOINTS;
	_pre_offboard_count = 0;
	_current_wp = 0;
	_state_start = hrt_absolute_time();

	ScheduleOnInterval(50_ms); // 20 Hz
	PX4_INFO("Starting square trajectory...");
	return true;
}

void SquareTrajectory::publish_offboard_control_mode()
{
	offboard_control_mode_s ocm{};
	ocm.position = true;
	ocm.velocity = false;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.timestamp = hrt_absolute_time();
	_offboard_control_mode_pub.publish(ocm);
}

void SquareTrajectory::publish_trajectory_setpoint(float x, float y, float z, float yaw)
{
	trajectory_setpoint_s sp{};
	sp.position[0] = x;
	sp.position[1] = y;
	sp.position[2] = z;
	sp.velocity[0] = NAN;
	sp.velocity[1] = NAN;
	sp.velocity[2] = NAN;
	sp.acceleration[0] = NAN;
	sp.acceleration[1] = NAN;
	sp.acceleration[2] = NAN;
	sp.yaw = yaw;
	sp.yawspeed = NAN;
	sp.timestamp = hrt_absolute_time();
	_trajectory_setpoint_pub.publish(sp);
}

void SquareTrajectory::send_vehicle_command(uint16_t cmd, float p1, float p2, float p3)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = p1;
	vcmd.param2 = p2;
	vcmd.param3 = p3;
	vcmd.param4 = NAN;
	vcmd.param5 = NAN;
	vcmd.param6 = NAN;
	vcmd.param7 = NAN;
	vcmd.target_system = 1;
	vcmd.target_component = 1;
	vcmd.source_system = 1;
	vcmd.source_component = 1;
	vcmd.from_external = false;
	vcmd.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(vcmd);
}

void SquareTrajectory::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Always publish offboard control mode and current setpoint to keep offboard alive
	if (_state != State::DONE && _state != State::IDLE && _state != State::LAND) {
		publish_offboard_control_mode();

		// Keep publishing current target
		if (_current_wp >= 0 && _current_wp < NUM_WAYPOINTS) {
			publish_trajectory_setpoint(
				_waypoints[_current_wp][0],
				_waypoints[_current_wp][1],
				_waypoints[_current_wp][2],
				_waypoints[_current_wp][3]);
		} else {
			// Default: hover at origin
			publish_trajectory_setpoint(0.0f, 0.0f, FLIGHT_HEIGHT, 0.0f);
		}
	}

	// Get current vehicle state
	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	vehicle_local_position_s local_pos{};
	_vehicle_local_position_sub.copy(&local_pos);

	switch (_state) {

	case State::IDLE:
		break;

	case State::SEND_SETPOINTS:
		// Must send setpoints for a while before switching to offboard
		_pre_offboard_count++;

		if (_pre_offboard_count >= PRE_OFFBOARD_SETPOINTS) {
			PX4_INFO("Pre-offboard setpoints sent (%d). Switching to OFFBOARD...", _pre_offboard_count);
			_state = State::OFFBOARD;
			_state_start = hrt_absolute_time();
		}

		break;

	case State::OFFBOARD:
		// Send OFFBOARD mode command
		// VEHICLE_CMD_DO_SET_MODE: param1=base_mode, param2=custom_main_mode
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f); // 6 = OFFBOARD

		if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			PX4_INFO("OFFBOARD mode confirmed. Arming...");
			_state = State::ARM;
			_state_start = hrt_absolute_time();

		} else if (hrt_elapsed_time(&_state_start) > 10_s) {
			PX4_ERR("Failed to enter OFFBOARD mode. Aborting.");
			_state = State::DONE;
		}

		break;

	case State::ARM:
		// VEHICLE_CMD_COMPONENT_ARM_DISARM: param1=1(arm), param2=21196(force)
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 21196.0f);

		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			PX4_INFO("Armed! Taking off to %.1fm...", (double)(-FLIGHT_HEIGHT));
			_current_wp = 0;
			_state = State::TAKEOFF;
			_state_start = hrt_absolute_time();

		} else if (hrt_elapsed_time(&_state_start) > 10_s) {
			PX4_ERR("Failed to arm. Aborting.");
			_state = State::DONE;
		}

		break;

	case State::TAKEOFF:
		// Wait to reach first waypoint (hover point)
		if (local_pos.xy_valid && local_pos.z_valid) {
			float dx = local_pos.x - _waypoints[0][0];
			float dy = local_pos.y - _waypoints[0][1];
			float dz = local_pos.z - _waypoints[0][2];
			float dist = sqrtf(dx * dx + dy * dy + dz * dz);

			if (dist < POSITION_THRESHOLD) {
				PX4_INFO("Takeoff complete. Starting square trajectory.");
				_current_wp = 1;
				_state = State::FLY_TO_WP;
				_state_start = hrt_absolute_time();

			} else if (hrt_elapsed_time(&_state_start) > 30_s) {
				PX4_ERR("Takeoff timeout. Landing.");
				_state = State::LAND;
			}
		}

		break;

	case State::FLY_TO_WP:
		if (local_pos.xy_valid && local_pos.z_valid) {
			float dx = local_pos.x - _waypoints[_current_wp][0];
			float dy = local_pos.y - _waypoints[_current_wp][1];
			float dz = local_pos.z - _waypoints[_current_wp][2];
			float dist = sqrtf(dx * dx + dy * dy + dz * dz);

			if (dist < POSITION_THRESHOLD) {
				PX4_INFO("[WP %d/%d] Reached (%.2f, %.2f, %.2f) yaw=%.0f deg. Holding %.1fs...",
					 _current_wp, NUM_WAYPOINTS - 1,
					 (double)local_pos.x, (double)local_pos.y, (double)local_pos.z,
					 (double)(math::degrees(_waypoints[_current_wp][3])),
					 (double)HOLD_TIME_S);
				_hold_start = hrt_absolute_time();
				_state = State::HOLD_AT_WP;

			} else if (hrt_elapsed_time(&_state_start) > 30_s) {
				PX4_WARN("[WP %d] Timeout (dist=%.2f). Moving on.", _current_wp, (double)dist);
				_hold_start = hrt_absolute_time();
				_state = State::HOLD_AT_WP;
			}
		}

		break;

	case State::HOLD_AT_WP:
		if (hrt_elapsed_time(&_hold_start) > (hrt_abstime)(HOLD_TIME_S * 1e6f)) {
			_current_wp++;

			if (_current_wp >= NUM_WAYPOINTS) {
				PX4_INFO("Square trajectory complete! Landing...");
				_state = State::LAND;

			} else {
				PX4_INFO("[WP %d/%d] Flying to (%.1f, %.1f) yaw=%.0f deg...",
					 _current_wp, NUM_WAYPOINTS - 1,
					 (double)_waypoints[_current_wp][0],
					 (double)_waypoints[_current_wp][1],
					 (double)(math::degrees(_waypoints[_current_wp][3])));
				_state = State::FLY_TO_WP;
				_state_start = hrt_absolute_time();
			}
		}

		break;

	case State::LAND:
		// Switch to AUTO.LAND: main_mode=4(AUTO), sub_mode=6(LAND)
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 6.0f);
		PX4_INFO("Land command sent.");
		_state = State::DONE;
		break;

	case State::DONE:
		PX4_INFO("Trajectory finished. Stopping module.");
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
}

int SquareTrajectory::task_spawn(int argc, char *argv[])
{
	SquareTrajectory *instance = new SquareTrajectory();

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

int SquareTrajectory::print_status()
{
	const char *state_str = "UNKNOWN";

	switch (_state) {
	case State::IDLE:           state_str = "IDLE"; break;
	case State::SEND_SETPOINTS: state_str = "SEND_SETPOINTS"; break;
	case State::OFFBOARD:       state_str = "OFFBOARD"; break;
	case State::ARM:            state_str = "ARM"; break;
	case State::TAKEOFF:        state_str = "TAKEOFF"; break;
	case State::FLY_TO_WP:      state_str = "FLY_TO_WP"; break;
	case State::HOLD_AT_WP:     state_str = "HOLD_AT_WP"; break;
	case State::LAND:           state_str = "LAND"; break;
	case State::DONE:           state_str = "DONE"; break;
	}

	PX4_INFO("State: %s, WP: %d/%d", state_str, _current_wp, NUM_WAYPOINTS - 1);
	return 0;
}

int SquareTrajectory::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SquareTrajectory::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Fly a 1m x 1m square trajectory at 1m altitude with yaw rotation at corners.

Uses OFFBOARD mode with position + yaw setpoints in NED frame.

Sequence:
  1. Pre-send setpoints (5s)
  2. Switch to OFFBOARD mode
  3. Arm (force)
  4. Fly square: (0,0) -> (1,0) -> (1,1) -> (0,1) -> (0,0)
  5. At each corner, rotate yaw to face next direction
  6. Land

### Usage
Start the trajectory:
$ square_trajectory start

Check status:
$ square_trajectory status

Stop:
$ square_trajectory stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("square_trajectory", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int square_trajectory_main(int argc, char *argv[])
{
	return SquareTrajectory::main(argc, argv);
}
