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
 * @file TrajectoryTest.hpp
 * @brief Trajectory test runner module
 *
 * Usage:
 *   trajectory_test start <name>   (e.g. square, circle)
 *   trajectory_test status
 *   trajectory_test stop
 */

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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <mathlib/mathlib.h>

#include "TrajectoryBase.hpp"

using namespace time_literals;

class TrajectoryTest : public ModuleBase<TrajectoryTest>, public px4::ScheduledWorkItem
{
public:
	TrajectoryTest();
	~TrajectoryTest() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

	/** Set which trajectory to fly (must be called before init) */
	void set_trajectory(TrajectoryBase *traj) { _trajectory = traj; }

	/** Store trajectory name from argv for task_spawn */
	static char s_trajectory_name[32];

private:
	void Run() override;

	void publish_trajectory_setpoint(const Waypoint &wp);
	void publish_offboard_control_mode();
	void send_vehicle_command(uint16_t cmd, float p1 = NAN, float p2 = NAN, float p3 = NAN);

	enum class State {
		IDLE,
		SEND_SETPOINTS,
		OFFBOARD,
		ARM,
		TAKEOFF,
		FLY_TO_WP,
		HOLD_AT_WP,
		LAND,
		DONE
	};

	State _state{State::IDLE};
	TrajectoryBase *_trajectory{nullptr};

	int _current_wp{0};
	int _pre_offboard_count{0};
	int _run_count{0};
	hrt_abstime _hold_start{0};
	hrt_abstime _state_start{0};
	hrt_abstime _last_cmd_time{0};
	hrt_abstime _last_log_time{0};

	static constexpr int PRE_OFFBOARD_SETPOINTS = 100; // 5s at 20Hz

	// uORB publications
	uORB::Publication<trajectory_setpoint_s>    _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s>  _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s>        _vehicle_command_pub{ORB_ID(vehicle_command)};

	// uORB subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
};
