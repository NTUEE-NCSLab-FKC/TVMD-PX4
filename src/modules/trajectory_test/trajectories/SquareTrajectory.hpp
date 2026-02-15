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
 * @file SquareTrajectory.hpp
 * @brief 1m x 1m square at 1m altitude with yaw rotation at corners
 *
 * Waypoint sequence:
 *   (0,0) yaw=0 -> (1,0) yaw=0 -> rotate 90
 *   (1,0) yaw=90 -> (1,1) yaw=90 -> rotate 180
 *   (1,1) yaw=180 -> (0,1) yaw=180 -> rotate -90
 *   (0,1) yaw=-90 -> (0,0) yaw=-90 -> rotate 0
 */

#pragma once

#include "../TrajectoryBase.hpp"

class SquareTrajectory : public TrajectoryBase
{
public:
	const char *name() const override { return "square"; }
	int num_waypoints() const override { return 9; }

	Waypoint waypoint(int index) const override
	{
		static constexpr float L = 1.0f;
		static constexpr float H = -1.0f; // 1m altitude (NED)
		static constexpr float HOLD = 2.0f;
		static constexpr float PI = M_PI_F;

		static const Waypoint wps[] = {
			{0, 0, H, 0,        HOLD},  // takeoff hover
			{L, 0, H, 0,        HOLD},  // corner 1
			{L, 0, H, PI / 2,   HOLD},  // rotate to 90
			{L, L, H, PI / 2,   HOLD},  // corner 2
			{L, L, H, PI,       HOLD},  // rotate to 180
			{0, L, H, PI,       HOLD},  // corner 3
			{0, L, H, -PI / 2,  HOLD},  // rotate to -90
			{0, 0, H, -PI / 2,  HOLD},  // corner 4
			{0, 0, H, 0,        HOLD},  // rotate back to 0
		};

		if (index >= 0 && index < num_waypoints()) {
			return wps[index];
		}

		return {0, 0, H, 0, HOLD};
	}
};
