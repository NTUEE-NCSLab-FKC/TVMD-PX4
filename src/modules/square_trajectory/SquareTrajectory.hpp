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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

using namespace time_literals;

class SquareTrajectory : public ModuleBase<SquareTrajectory>, public px4::ScheduledWorkItem
{
public:
	SquareTrajectory();
	~SquareTrajectory() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

private:
	void Run() override;

	void publish_trajectory_setpoint(float x, float y, float z, float yaw);
	void publish_offboard_control_mode();
	void send_vehicle_command(uint16_t cmd, float p1 = NAN, float p2 = NAN, float p3 = NAN);

	enum class State {
		IDLE,
		SEND_SETPOINTS,   // pre-send setpoints before switching to offboard
		OFFBOARD,         // switch to offboard mode
		ARM,              // arm vehicle
		TAKEOFF,          // fly to initial hover point
		FLY_TO_WP,        // fly to next waypoint
		HOLD_AT_WP,       // hold position at waypoint
		LAND,             // switch to land mode
		DONE
	};

	State _state{State::IDLE};

	// Square trajectory configuration
	static constexpr float SIDE_LENGTH = 1.0f;
	static constexpr float FLIGHT_HEIGHT = -1.0f; // NED: negative = up
	static constexpr float POSITION_THRESHOLD = 0.3f;
	static constexpr float HOLD_TIME_S = 2.0f;
	static constexpr int PRE_OFFBOARD_SETPOINTS = 100; // ~5s at 20Hz

	// Waypoints: {x, y, z, yaw}
	// Square with yaw rotation at corners
	static constexpr int NUM_WAYPOINTS = 9;
	float _waypoints[NUM_WAYPOINTS][4] = {
		{0.0f,          0.0f,          FLIGHT_HEIGHT, 0.0f},                // takeoff
		{SIDE_LENGTH,   0.0f,          FLIGHT_HEIGHT, 0.0f},                // corner 1 (yaw=0)
		{SIDE_LENGTH,   0.0f,          FLIGHT_HEIGHT, M_PI_F / 2.0f},      // rotate to 90
		{SIDE_LENGTH,   SIDE_LENGTH,   FLIGHT_HEIGHT, M_PI_F / 2.0f},      // corner 2 (yaw=90)
		{SIDE_LENGTH,   SIDE_LENGTH,   FLIGHT_HEIGHT, M_PI_F},             // rotate to 180
		{0.0f,          SIDE_LENGTH,   FLIGHT_HEIGHT, M_PI_F},             // corner 3 (yaw=180)
		{0.0f,          SIDE_LENGTH,   FLIGHT_HEIGHT, -M_PI_F / 2.0f},    // rotate to -90
		{0.0f,          0.0f,          FLIGHT_HEIGHT, -M_PI_F / 2.0f},    // corner 4 (yaw=-90)
		{0.0f,          0.0f,          FLIGHT_HEIGHT, 0.0f},               // rotate back to 0
	};

	int _current_wp{0};
	int _pre_offboard_count{0};
	hrt_abstime _hold_start{0};
	hrt_abstime _state_start{0};

	// uORB publications
	uORB::Publication<trajectory_setpoint_s>    _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s>  _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s>        _vehicle_command_pub{ORB_ID(vehicle_command)};

	// uORB subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
};
