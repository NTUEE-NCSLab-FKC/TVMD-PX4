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

#include "TrajectoryTest.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <string.h>

// Include all trajectory definitions
#include "trajectories/SquareTrajectory.hpp"
#include "trajectories/CircleTrajectory.hpp"
#include "trajectories/HoverTrajectory.hpp"

// Static storage for trajectory name passed via argv
char TrajectoryTest::s_trajectory_name[32] = {};

/**
 * @brief Create trajectory by name
 * @return Heap-allocated trajectory, or nullptr if not found
 *
 * To add a new trajectory:
 *   1. Create trajectories/MyTrajectory.hpp
 *   2. #include it above
 *   3. Add an else-if branch below
 */
static TrajectoryBase *create_trajectory(const char *name)
{
	if (strcmp(name, "square") == 0) {
		return new SquareTrajectory();

	} else if (strcmp(name, "circle") == 0) {
		return new CircleTrajectory();

	} else if (strcmp(name, "hover") == 0) {
		return new HoverTrajectory();
	}

	// Add new trajectories here:
	// } else if (strcmp(name, "figure8") == 0) {
	//     return new FigureEightTrajectory();
	// }

	return nullptr;
}

TrajectoryTest::TrajectoryTest() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

TrajectoryTest::~TrajectoryTest()
{
	delete _trajectory;
}

bool TrajectoryTest::init()
{
	if (!_trajectory) {
		PX4_ERR("No trajectory set");
		return false;
	}

	_state = State::SEND_SETPOINTS;
	_pre_offboard_count = 0;
	_current_wp = 0;
	_land_count = 0;
	_run_count = 0;
	_state_start = hrt_absolute_time();
	_last_log_time = hrt_absolute_time();

	ScheduleOnInterval(50_ms); // 20 Hz
	PX4_INFO("Starting trajectory: %s (%d waypoints)", _trajectory->name(), _trajectory->num_waypoints());
	return true;
}

void TrajectoryTest::publish_offboard_control_mode()
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

void TrajectoryTest::publish_trajectory_setpoint(const Waypoint &wp)
{
	trajectory_setpoint_s sp{};
	sp.position[0] = wp.x;
	sp.position[1] = wp.y;
	sp.position[2] = wp.z;
	sp.velocity[0] = NAN;
	sp.velocity[1] = NAN;
	sp.velocity[2] = NAN;
	sp.acceleration[0] = NAN;
	sp.acceleration[1] = NAN;
	sp.acceleration[2] = NAN;
	sp.yaw = wp.yaw;
	sp.yawspeed = NAN;
	sp.timestamp = hrt_absolute_time();
	_trajectory_setpoint_pub.publish(sp);
}

void TrajectoryTest::send_vehicle_command(uint16_t cmd, float p1, float p2, float p3)
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

void TrajectoryTest::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	_run_count++;

	const int num_wps = _trajectory->num_waypoints();

	// Keep offboard alive — publish in ALL active states including LAND
	if (_state != State::DONE && _state != State::IDLE) {
		publish_offboard_control_mode();

		int wp_idx = (_current_wp >= 0 && _current_wp < num_wps) ? _current_wp : 0;
		publish_trajectory_setpoint(_trajectory->waypoint(wp_idx));
	}

	// Read vehicle state
	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	vehicle_local_position_s local_pos{};
	_vehicle_local_position_sub.copy(&local_pos);

	// Periodic heartbeat log (every 5s) for hardware diagnostics
	if (hrt_elapsed_time(&_last_log_time) > 5000000) {
		_last_log_time = hrt_absolute_time();
		PX4_INFO("[heartbeat] run=%d state=%d wp=%d/%d nav=%d arm=%d pos_valid=%d",
			 _run_count, (int)_state, _current_wp, num_wps,
			 vehicle_status.nav_state, vehicle_status.arming_state,
			 (local_pos.xy_valid && local_pos.z_valid) ? 1 : 0);
	}

	switch (_state) {

	case State::IDLE:
		break;

	case State::SEND_SETPOINTS:
		_pre_offboard_count++;

		if (_pre_offboard_count >= PRE_OFFBOARD_SETPOINTS) {
			PX4_INFO("Pre-offboard done (%d setpoints). Switching to OFFBOARD...", _pre_offboard_count);
			_state = State::OFFBOARD;
			_state_start = hrt_absolute_time();
		}

		break;

	case State::OFFBOARD:
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f); // 6=OFFBOARD

		if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			PX4_INFO("OFFBOARD confirmed. Arming...");
			_state = State::ARM;
			_state_start = hrt_absolute_time();

		} else if (hrt_elapsed_time(&_state_start) > 10_s) {
			PX4_ERR("OFFBOARD timeout. Aborting.");
			_state = State::DONE;
		}

		break;

	case State::ARM:
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 21196.0f);

		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			PX4_INFO("Armed! Taking off...");
			_current_wp = 0;
			_state = State::TAKEOFF;
			_state_start = hrt_absolute_time();

		} else if (hrt_elapsed_time(&_state_start) > 10_s) {
			PX4_ERR("Arm timeout. Aborting.");
			_state = State::DONE;
		}

		break;

	case State::TAKEOFF: {
		if (local_pos.xy_valid && local_pos.z_valid) {
			Waypoint wp0 = _trajectory->waypoint(0);
			float dx = local_pos.x - wp0.x;
			float dy = local_pos.y - wp0.y;
			float dz = local_pos.z - wp0.z;
			float dist = sqrtf(dx * dx + dy * dy + dz * dz);

			if (dist < _trajectory->position_threshold()) {
				PX4_INFO("Takeoff complete. Starting %s trajectory.", _trajectory->name());
				_current_wp = 1;
				_state = State::FLY_TO_WP;
				_state_start = hrt_absolute_time();

			} else if (hrt_elapsed_time(&_state_start) > 30_s) {
				PX4_ERR("Takeoff timeout. Landing.");
				_state = State::LAND;
			}
		}

		break;
	}

	case State::FLY_TO_WP: {
		if (local_pos.xy_valid && local_pos.z_valid) {
			Waypoint wp = _trajectory->waypoint(_current_wp);
			float dx = local_pos.x - wp.x;
			float dy = local_pos.y - wp.y;
			float dz = local_pos.z - wp.z;
			float dist = sqrtf(dx * dx + dy * dy + dz * dz);

			if (dist < _trajectory->position_threshold()) {
				PX4_INFO("[WP %d/%d] Reached (%.2f, %.2f, %.2f) yaw=%.0f deg",
					 _current_wp, num_wps - 1,
					 (double)local_pos.x, (double)local_pos.y, (double)local_pos.z,
					 (double)math::degrees(wp.yaw));
				_hold_start = hrt_absolute_time();
				_state = State::HOLD_AT_WP;

			} else if (hrt_elapsed_time(&_state_start) > (hrt_abstime)(_trajectory->waypoint_timeout() * 1e6f)) {
				PX4_WARN("[WP %d] Timeout (dist=%.2f). Continuing.", _current_wp, (double)dist);
				_hold_start = hrt_absolute_time();
				_state = State::HOLD_AT_WP;
			}
		}

		break;
	}

	case State::HOLD_AT_WP: {
		Waypoint wp = _trajectory->waypoint(_current_wp);
		hrt_abstime hold_us = (hrt_abstime)(wp.hold_time * 1e6f);

		if (hrt_elapsed_time(&_hold_start) > hold_us) {
			_current_wp++;

			if (_current_wp >= num_wps) {
				PX4_INFO("%s trajectory complete! Landing...", _trajectory->name());
				_state = State::LAND;

			} else {
				Waypoint next = _trajectory->waypoint(_current_wp);
				PX4_INFO("[WP %d/%d] -> (%.2f, %.2f) yaw=%.0f deg",
					 _current_wp, num_wps - 1,
					 (double)next.x, (double)next.y,
					 (double)math::degrees(next.yaw));
				_state = State::FLY_TO_WP;
				_state_start = hrt_absolute_time();
			}
		}

		break;
	}

	case State::LAND:
		// AUTO.LAND: main_mode=4(AUTO), sub_mode=6(LAND)
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
		_land_count++;

		if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
		    || _land_count > LAND_REPEAT_COUNT) {
			PX4_INFO("Land %s after %d attempts.",
				 (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)
				 ? "confirmed" : "timeout", _land_count);
			_state = State::DONE;
		}

		break;

	case State::DONE:
		PX4_INFO("Done. Module stopping.");
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
}

int TrajectoryTest::task_spawn(int argc, char *argv[])
{
	// Parse trajectory name from argv
	const char *traj_name = "square"; // default

	// argv: ["start", "square"] or just ["start"]
	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "start") != 0 &&
		    strcmp(argv[i], "stop") != 0 &&
		    strcmp(argv[i], "status") != 0) {
			traj_name = argv[i];
			break;
		}
	}

	strncpy(s_trajectory_name, traj_name, sizeof(s_trajectory_name) - 1);
	s_trajectory_name[sizeof(s_trajectory_name) - 1] = '\0';

	TrajectoryBase *trajectory = create_trajectory(s_trajectory_name);

	if (!trajectory) {
		PX4_ERR("Unknown trajectory: '%s'. Available: square, circle, hover", s_trajectory_name);
		return PX4_ERROR;
	}

	TrajectoryTest *instance = new TrajectoryTest();

	if (instance) {
		instance->set_trajectory(trajectory);
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
		delete trajectory;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int TrajectoryTest::print_status()
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

	PX4_INFO("Trajectory: %s | State: %s | WP: %d/%d",
		 _trajectory ? _trajectory->name() : "none",
		 state_str,
		 _current_wp,
		 _trajectory ? _trajectory->num_waypoints() - 1 : 0);
	return 0;
}

int TrajectoryTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TrajectoryTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Trajectory test runner. Flies predefined trajectories using OFFBOARD mode.

Available trajectories:
  square   - 1m x 1m square with yaw rotation at corners
  circle   - 1m radius circle with tangent yaw
  hover    - hover in place at 0.5m for 10s

### Usage
$ trajectory_test start square
$ trajectory_test start circle
$ trajectory_test status
$ trajectory_test stop

### Adding new trajectories
1. Create trajectories/MyTrajectory.hpp inheriting TrajectoryBase
2. Include and register in TrajectoryTest.cpp create_trajectory()
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("trajectory_test", "modules");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start trajectory (default: square)");
	PRINT_MODULE_USAGE_ARG("square|circle|hover", "Trajectory name", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int trajectory_test_main(int argc, char *argv[])
{
	return TrajectoryTest::main(argc, argv);
}
