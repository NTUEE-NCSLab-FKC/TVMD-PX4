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
 * @file CircleTrajectory.hpp
 * @brief Circle with radius 1m at 1m altitude, 16 waypoints
 *
 * Circle center at (1, 0). Yaw always faces tangent direction.
 * Starts and ends at (0, 0).
 */

#pragma once

#include "../TrajectoryBase.hpp"
#include <math.h>

class CircleTrajectory : public TrajectoryBase
{
public:
	const char *name() const override { return "circle"; }

	int num_waypoints() const override
	{
		return NUM_CIRCLE_POINTS + 1; // +1 for return to start
	}

	Waypoint waypoint(int index) const override
	{
		if (index < 0 || index >= num_waypoints()) {
			return {0, 0, HEIGHT, 0, HOLD};
		}

		if (index == NUM_CIRCLE_POINTS) {
			// Last point: return to start
			return {0, 0, HEIGHT, 0, HOLD};
		}

		// Circle parametric: center at (RADIUS, 0)
		// angle goes from PI to PI (full circle starting from (0,0))
		float angle = M_PI_F + (2.0f * M_PI_F * index) / NUM_CIRCLE_POINTS;
		float x = CENTER_X + RADIUS * cosf(angle);
		float y = CENTER_Y + RADIUS * sinf(angle);

		// Yaw = tangent direction (perpendicular to radius, in direction of travel)
		float yaw = angle + M_PI_F / 2.0f;

		// Normalize yaw to [-PI, PI]
		while (yaw > M_PI_F) { yaw -= 2.0f * M_PI_F; }
		while (yaw < -M_PI_F) { yaw += 2.0f * M_PI_F; }

		return {x, y, HEIGHT, yaw, HOLD};
	}

private:
	static constexpr int NUM_CIRCLE_POINTS = 16;
	static constexpr float RADIUS = 1.0f;
	static constexpr float CENTER_X = 1.0f; // circle center so start is at (0,0)
	static constexpr float CENTER_Y = 0.0f;
	static constexpr float HEIGHT = -1.0f;
	static constexpr float HOLD = 1.0f;
};
