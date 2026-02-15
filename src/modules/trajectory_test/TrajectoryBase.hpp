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

#include <math.h>

/**
 * @brief A single waypoint in NED frame
 */
struct Waypoint {
	float x;          ///< North (m)
	float y;          ///< East (m)
	float z;          ///< Down (m), negative = up
	float yaw;        ///< Heading (rad)
	float hold_time;  ///< Time to hold at this waypoint (s)
};

/**
 * @brief Base class for all trajectory definitions.
 *
 * To add a new trajectory:
 *   1. Create a new file in trajectories/ (e.g. MyTrajectory.hpp)
 *   2. Inherit from TrajectoryBase
 *   3. Implement name(), num_waypoints(), waypoint()
 *   4. Register in TrajectoryTest.cpp create_trajectory()
 */
class TrajectoryBase
{
public:
	virtual ~TrajectoryBase() = default;

	/** @return Human-readable trajectory name */
	virtual const char *name() const = 0;

	/** @return Total number of waypoints */
	virtual int num_waypoints() const = 0;

	/**
	 * @brief Get waypoint at index
	 * @param index Waypoint index [0, num_waypoints()-1]
	 * @return Waypoint in NED frame
	 */
	virtual Waypoint waypoint(int index) const = 0;

	/** @return Position threshold to consider waypoint reached (m) */
	virtual float position_threshold() const { return 0.3f; }

	/** @return Timeout per waypoint (s) */
	virtual float waypoint_timeout() const { return 30.0f; }
};
