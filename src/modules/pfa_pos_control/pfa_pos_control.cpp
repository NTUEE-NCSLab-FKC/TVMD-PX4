/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the uuv position control module and is designed for
 * partially fully actuated systems.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

#include "pfa_pos_control.hpp"



/**
 * PFA pos_controller app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int pfa_pos_control_main(int argc, char *argv[]);


PFAPOSControl::PFAPOSControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_takeoff_status_pub.advertise();
}

PFAPOSControl::~PFAPOSControl()
{
	perf_free(_loop_perf);
}

bool PFAPOSControl::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void PFAPOSControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		// The original definition: initial_v = -g / velociry_p_gain,
		// Here, we let the initial thrust be 0.1f
		_takeoff.generateInitialRampValue(10.0f * CONSTANTS_ONE_G);
	}
}

/**
 * Record the time when position control is enabled, which is used to
 * schedule the takeoff ramp.
 */
void PFAPOSControl::_update_position_control_enable_time()
{
	if (_vcontrol_mode_sub.updated()) {
		const bool previous_vcontrol_mode_enabled = _vcontrol_mode.flag_multicopter_position_control_enabled;
		if (_vcontrol_mode_sub.update(&_vcontrol_mode)) {
			if (!previous_vcontrol_mode_enabled && _vcontrol_mode.flag_multicopter_position_control_enabled) {
				_time_position_control_enabled = _vcontrol_mode.timestamp;
			}
		}
	}
}


/**
 * Publish attitude setpoint.
 */
void PFAPOSControl::publish_attitude_setpoint(
	const Vector3f &thrust_sp, const Vector3f &euler_attitude_des)
{
	//watch if inputs are not to high
	vehicle_attitude_setpoint_s vehicle_attitude_setpoint = {};
	vehicle_attitude_setpoint.timestamp = hrt_absolute_time();

	vehicle_attitude_setpoint.roll_body = euler_attitude_des(0);
	vehicle_attitude_setpoint.pitch_body = euler_attitude_des(1);
	vehicle_attitude_setpoint.yaw_body = euler_attitude_des(2);

	// q_d is what MAVLink ATTITUDE_TARGET reports; without it GCS / scripts see 0 deg
	Quatf(Eulerf(euler_attitude_des(0), euler_attitude_des(1), euler_attitude_des(2))).copyTo(vehicle_attitude_setpoint.q_d);

	const Vector3f output_thrust = _checkAllFinite(thrust_sp);
	output_thrust.copyTo(vehicle_attitude_setpoint.thrust_body);
	_att_sp_pub.publish(vehicle_attitude_setpoint);
}


/**
 * Calculate the desired thrust from the given trajectory setpoint.
 * If any entry of the setpoint is NaN, the corresponding control input
 * will be neglected.
 */
void PFAPOSControl::pose_controller_6dof(
		const trajectory_setpoint_s &traj_des, const Vector3f &euler_attitude_des,
		const vehicle_attitude_s &vehicle_attitude, const vehicle_local_position_s &vlocal_pos)
{
	// Retrieve gains
	const Vector3f Kp = Vector3f(_param_pose_gain_x.get(), _param_pose_gain_y.get(), _param_pose_gain_z.get());
	const Vector3f Kv = Vector3f(_param_pose_gain_d_x.get(), _param_pose_gain_d_y.get(), _param_pose_gain_d_z.get());

	// Retrieve current states
	// Quaternion rotation from the FRD body frame to the NED earth frame
	const Quatf q_att(vehicle_attitude.q);
	const Vector3f pos = Vector3f(vlocal_pos.x, vlocal_pos.y, vlocal_pos.z);
	const Vector3f vel = Vector3f(vlocal_pos.vx, vlocal_pos.vy, vlocal_pos.vz);

	// Calculate constrained velocity setpoint
	// TODO: the position setpoint should be constrained as the velocity setpoint
	// is saturated.
	// NaN components are handled per axis: a NaN position component disables the
	// position feedback of that axis only, a NaN velocity component means "zero
	// velocity" (damping) on that axis. Zeroing the whole vector when a single
	// component is NaN (e.g. flight tasks set z = NaN while the throttle stick is
	// off-centre) would silently drop the position hold of the other axes.
	const Vector3f vel_des = _zeroNonFinite(Vector3f(traj_des.velocity));
	const Vector3f limited_vel_des = matrix::constrain(vel_des,
		Vector3f(-_speed_xy_max, -_speed_xy_max, _speed_up_max),
		Vector3f(_speed_xy_max, _speed_xy_max, _speed_dn_max));

	// Calculate errors
	const Vector3f err_pos = _zeroNonFinite(Vector3f(traj_des.position) - pos);
	const Vector3f err_vel = _zeroNonFinite(limited_vel_des - vel);
	const Vector3f err_acc = _zeroNonFinite(Vector3f(traj_des.acceleration));

	// NED frame (north east down)
	const Vector3f resist_gravity = Vector3f(0, 0, -CONSTANTS_ONE_G);

	// Calcuate the control input
	const Vector3f control_acc = err_acc + resist_gravity + Kv.emult(err_vel) + Kp.emult(err_pos);
	const Vector3f control_force = control_acc * _param_vehicle_mass.get();
	const Vector3f control_normalized_force = control_force / _param_vehicle_max_thrust.get();

	// Attitude yield (PFA_ATT_YIELD): if the commanded attitude cannot deliver the
	// desired force (force outside the thrust cone), tilt the attitude setpoint
	// towards the force until it fits, instead of letting the position run away.
	const Vector3f nominal_force = (err_acc + resist_gravity) * _param_vehicle_mass.get()
				       / _param_vehicle_max_thrust.get();
	const Vector3f euler_cmd = attitude_yield(nominal_force, control_normalized_force, euler_attitude_des, _dt);

	// Rotate the thrust from global to body frame
	const Vector3f rotated_input = q_att.rotateVectorInverse(control_normalized_force);
	// const Vector3f rotated_input = control_normalized_force;

	// Limit thrust vector and check for saturation
	// NED frame (north east down)
	Vector3f limited_thrust_des = matrix::constrain(rotated_input,
		Vector3f(-_thrust_xy_max, -_thrust_xy_max, _thrust_up_max),
		Vector3f(_thrust_xy_max, _thrust_xy_max, _thrust_min));

	// Vertical priority (PFA_VERT_PRIO): the body z computed above assumes the full
	// body x/y demand is delivered, because part of the world-frame lift comes from
	// the body x/y components when the vehicle is tilted. If x/y were clipped, that
	// part is lost and the vehicle sinks (and the position error that caused the
	// clipping keeps growing). Re-solve body z so that the world-frame vertical force
	// still equals the demand, and accept the horizontal shortfall instead:
	//   world_z = R(2,0)*bx + R(2,1)*by + R(2,2)*bz  ->  bz = (F_z - R20*bx - R21*by) / R22
	bool xy_clipped = fabsf(limited_thrust_des(0) - rotated_input(0)) > FLT_EPSILON
			  || fabsf(limited_thrust_des(1) - rotated_input(1)) > FLT_EPSILON;

	const Dcmf R(q_att);	// body FRD -> NED
	const float r22 = R(2, 2);	// cos(roll) * cos(pitch)

	// Both corrections assume the body z axis points mostly up; beyond 60 deg of tilt
	// (e.g. during a flip) more body lift would mainly push the vehicle sideways.
	const auto vertical_priority = [&]() {
		if (!xy_clipped || r22 <= 0.5f) {
			return;
		}

		if (_param_vert_prio.get() > 0) {
			const float bz = (control_normalized_force(2)
					  - R(2, 0) * limited_thrust_des(0)
					  - R(2, 1) * limited_thrust_des(1)) / r22;
			limited_thrust_des(2) = math::constrain(bz, _thrust_up_max, _thrust_min);
		}

		// Lift floor (PFA_FUP_MIN): while saturated, never let the body lift drop
		// below a fraction of the hover thrust. Only applied while x/y are clipped,
		// so normal descents and landing are not affected, and never above the
		// current takeoff ramp limit (_thrust_up_max).
		const float fup_min = _param_fup_min.get();

		if (fup_min > 0.f) {
			const float hover_z = -CONSTANTS_ONE_G * _param_vehicle_mass.get()
					      / _param_vehicle_max_thrust.get();	// < 0 (NED, up)
			const float floor_z = math::max(fup_min * hover_z, _thrust_up_max);
			limited_thrust_des(2) = math::min(limited_thrust_des(2), floor_z);
		}
	};

	vertical_priority();

	// Thrust cone: the servos can tilt the thrust by at most servo_tilt_max(), so the
	// body x/y force the allocator can deliver scales with the body z force. Limit the
	// x/y magnitude (direction kept) to tan(servo max) * |b_z| on top of the fixed box.
	// Uses the body z after the vertical priority: when strongly tilted the raw body z
	// demand can be ~0 (or positive), which would wrongly collapse the x/y force.
	if (_param_att_yield.get() > 0) {
		const float xy_cone = tanf(servo_tilt_max()) * fabsf(limited_thrust_des(2));
		const float xy_norm = matrix::Vector2f(limited_thrust_des(0), limited_thrust_des(1)).norm();

		if (xy_norm > xy_cone && xy_norm > FLT_EPSILON) {
			limited_thrust_des(0) *= xy_cone / xy_norm;
			limited_thrust_des(1) *= xy_cone / xy_norm;
			xy_clipped = true;
			vertical_priority();	// re-solve body z for the reduced x/y
		}
	}

	publish_attitude_setpoint(limited_thrust_des, euler_cmd);

	vehicle_local_position_setpoint_s local_pos_sp{};
	local_pos_sp.timestamp = hrt_absolute_time();
	local_pos_sp.x = err_pos(0);
	local_pos_sp.y = err_pos(1);
	local_pos_sp.z = err_pos(2);
	local_pos_sp.vx = limited_vel_des(0);
	local_pos_sp.vy = limited_vel_des(1);
	local_pos_sp.vz = limited_vel_des(2);
	control_acc.copyTo(local_pos_sp.acceleration);
	control_normalized_force.copyTo(local_pos_sp.thrust);

	_local_pos_sp_pub.publish(local_pos_sp);
}


float PFAPOSControl::servo_tilt_max() const
{
	const float servo_max = math::min(_param_servo_x_angle_max.get(), _param_servo_y_angle_max.get());
	return math::radians((servo_max > 1.f && servo_max < 89.f) ? servo_max : 30.f);
}

/**
 * Attitude yield.
 *
 * The thrust of the commanded attitude R_d points along t_d = R_d * (0, 0, -1).
 * Stage 1: if the nominal force (gravity + acceleration feedforward) is more than sigma
 * (PFA_YIELD_TILT) away from t_d, the commanded tilt is not feasible; rotate about
 * t_d x F by theta - sigma (closed form, smallest rotation) so the force sits on the cone.
 * Stage 2: if the low-passed full force (incl. position feedback) is still more than the
 * servo range + YIELD_DRIFT_MARGIN_DEG away from that attitude, the vehicle is being pushed
 * away: lean further into the force, capped at MPC_TILTMAX_AIR.
 * Stage 1 follows the command exactly; the stage-2 angle grows immediately and shrinks at
 * most at PFA_YIELD_RATE. roll / pitch are
 * then solved so that the thrust axis matches the rotated one with the commanded yaw.
 */
Vector3f PFAPOSControl::attitude_yield(const Vector3f &force_nominal, const Vector3f &force_ned,
				       const Vector3f &euler_des, float dt)
{
	const Dcmf R_d{Eulerf(euler_des(0), euler_des(1), euler_des(2))};
	const Vector3f t_d = R_d * Vector3f(0.f, 0.f, -1.f);
	const float hover_norm = CONSTANTS_ONE_G * _param_vehicle_mass.get() / _param_vehicle_max_thrust.get();
	const float step = math::radians(_param_yield_rate.get()) * dt;

	// Low-passed full force (incl. position / velocity feedback) for the drift stage
	if (!_yield_force_lp_valid || !_yield_force_lp.isAllFinite()) {
		_yield_force_lp = force_ned;
		_yield_force_lp_valid = true;

	} else {
		const float alpha = math::constrain(dt / (YIELD_FORCE_TAU + dt), 0.f, 1.f);
		_yield_force_lp += (force_ned - _yield_force_lp) * alpha;
	}

	// angle between thrust axis t and force f (axis updated only if well defined)
	const auto angle_to = [&](const Vector3f &t, const Vector3f &f, Vector3f &axis) {
		const float fn = f.norm();

		if (!PX4_ISFINITE(fn) || fn < 0.3f * hover_norm) {
			return 0.f;
		}

		const Vector3f n = t.cross(f / fn);

		if (n.norm() > 1e-6f) {
			axis = n.normalized();
		}

		return atan2f(n.norm(), t.dot(f / fn));	// accurate also near 0 (acosf is not)
	};

	// Stage 2 only: growing is immediate (a slower yield arrives after the vehicle has already
	// been pushed away); the recovery towards the stage-1 attitude is rate limited
	// (PFA_YIELD_RATE), because the low-passed full force can change quickly.
	const auto update = [&](float &angle, float target) {
		angle = (target >= angle) ? target : math::max(target, angle - step);
		angle = math::constrain(angle, 0.f, M_PI_F);
	};

	// Only a commanded attitude that is itself upside down (flip manoeuvres) is left alone.
	const bool valid = (_param_att_yield.get() > 0) && (t_d(2) < 0.f);

	// Stage 1 - feasibility of the commanded attitude: nominal force (gravity + acceleration
	// feedforward) outside the PFA_YIELD_TILT cone. The position feedback is deliberately not
	// used here: attitude -> position error -> force -> yield -> attitude is a loop, and with the
	// full force the setpoint swung between ~6 and ~43 deg in SITL (Kp 50 / Kd 25).
	// No rate limit here: stage 1 depends only on the command and the nominal force, so it is
	// already as smooth as the command. A rate-limited recovery made a command step-down
	// (e.g. 52 -> 30 deg, typical when PFA_DES_* arrives late) drop the setpoint to ~0 deg and
	// ramp back to the cone, i.e. yield far more than needed (SITL log 17_09_35).
	const float theta1 = valid ? angle_to(t_d, force_nominal, _yield_axis) : 0.f;
	_yield_angle = valid ? math::max(theta1 - math::radians(_param_yield_tilt.get()), 0.f) : 0.f;
	const float phi1 = _yield_angle;	// = theta1 - sigma <= theta1: never past the force
	const Vector3f t_1 = Dcmf(matrix::AxisAnglef(_yield_axis, phi1)) * t_d;

	// Stage 2 - drift: the (low-passed) full force is far outside even the servo range of the
	// stage-1 attitude, i.e. the vehicle is being pushed away (large error, a tilt the yield
	// came too late for). Lean into it like a multicopter. The margin keeps normal position
	// corrections (which the servos absorb or which only clip briefly) out of this stage.
	const float theta2 = valid ? angle_to(t_1, _yield_force_lp, _yield_axis2) : 0.f;
	const float drift_threshold = servo_tilt_max() + math::radians(YIELD_DRIFT_MARGIN_DEG);
	update(_yield_angle2, valid ? math::max(theta2 - drift_threshold, 0.f) : 0.f);
	float phi2 = valid ? math::min(_yield_angle2, theta2) : _yield_angle2;

	if (phi1 < 1e-4f && phi2 < 1e-4f) {
		return euler_des;
	}

	// Tilt cap on stage 2: like a multicopter, do not lean further than MPC_TILTMAX_AIR
	// because of position errors (a commanded tilt above that is not increased either).
	const auto tilt_of = [&](float a) {
		const Vector3f t = Dcmf(matrix::AxisAnglef(_yield_axis2, a)) * t_1;
		return acosf(math::constrain(-t(2), -1.f, 1.f));
	};
	const float tilt_cap = math::max(math::radians(_param_mpc_tiltmax_air.get()), tilt_of(0.f));

	if (phi2 > 0.f && tilt_of(phi2) > tilt_cap) {
		float lo = 0.f, hi = phi2;

		for (int i = 0; i < 16; i++) {
			const float mid = 0.5f * (lo + hi);
			(tilt_of(mid) > tilt_cap ? hi : lo) = mid;
		}

		phi2 = lo;
	}

	const Vector3f t_y = Dcmf(matrix::AxisAnglef(_yield_axis2, phi2)) * t_1;

	// roll / pitch for the commanded yaw: in the yaw frame the body z axis of
	// R = Rz(yaw) Ry(pitch) Rx(roll) is (cos(roll) sin(pitch), -sin(roll), cos(roll) cos(pitch))
	const float yaw = euler_des(2);
	const Vector3f z_b = -(Dcmf(Eulerf(0.f, 0.f, -yaw)) * t_y);
	const float roll = asinf(math::constrain(-z_b(1), -1.f, 1.f));
	const float pitch = atan2f(z_b(0), z_b(2));

	return Vector3f(roll, pitch, yaw);
}

/**
 * Calculate the desired thrust that hovers the vehicle at the current
 * position.
 */
void PFAPOSControl::stabilization_controller_6dof(
		const trajectory_setpoint_s &traj_des, const Vector3f &euler_attitude_des,
		const vehicle_attitude_s &vehicle_attitude, const vehicle_local_position_s &vlocal_pos)
{
	trajectory_setpoint_s stab_traj_des = traj_des;
	stab_traj_des.position[0] = 0;
	stab_traj_des.position[1] = 0;
	stab_traj_des.position[2] = vlocal_pos.z;
	stab_traj_des.velocity[0] = 0;
	stab_traj_des.velocity[1] = 0;
	stab_traj_des.velocity[2] = 0;
	stab_traj_des.acceleration[0] = 0;
	stab_traj_des.acceleration[1] = 0;
	stab_traj_des.acceleration[2] = 2.5f;	// -g + 4.5 ~ -5.0

	pose_controller_6dof(stab_traj_des, euler_attitude_des, vehicle_attitude, vlocal_pos);
}

/**
 * Yaw setpoint for the position branch.
 * - finite trajectory yaw: use it (offboard scripts, POSCTL with the yaw stick released)
 * - yaw NaN but yawspeed finite: integrate the yaw rate (POSCTL while the yaw stick is
 *   moved; before, the target fell back to the current yaw and the stick had no effect)
 * - both NaN: current yaw, i.e. yaw is not controlled (unchanged behaviour)
 */
float PFAPOSControl::position_yaw_setpoint(const trajectory_setpoint_s &traj_sp, const float dt)
{
	const float yaw_now = Eulerf(Quatf(_vehicle_attitude.q))(2);

	if (PX4_ISFINITE(traj_sp.yaw)) {
		_pos_yaw_sp = traj_sp.yaw;
		_pos_yaw_valid = true;

	} else if (PX4_ISFINITE(traj_sp.yawspeed)) {
		if (!_pos_yaw_valid) {
			_pos_yaw_sp = yaw_now;
			_pos_yaw_valid = true;
		}

		_pos_yaw_sp = matrix::wrap_pi(_pos_yaw_sp + traj_sp.yawspeed * dt);

	} else {
		_pos_yaw_sp = yaw_now;
		_pos_yaw_valid = false;
	}

	return _pos_yaw_sp;
}

/**
 * Desired roll and pitch.
 *
 * PFA_ATT_SRC = 0: PFA_DES_ROLL / PFA_DES_PITCH (unchanged behaviour).
 * PFA_ATT_SRC = 1: MAVLink DEBUG_VECT named "att_cmd" with x = roll (deg), y = pitch (deg),
 *   z = sender time of the sample (s). DEBUG_VECT has only three fields, so the rate is the
 *   difference of the last two samples over their sender times: arrival jitter does not
 *   enter, and a lost packet just makes the difference span two periods. Between packets
 *   the command is extrapolated, theta = theta_k + rate * age, with age capped at
 *   PFA_ATT_EXTR, so link holds no longer freeze the command into a staircase. Without a
 *   new sample for PFA_ATT_TOUT the parameters take over again.
 * Each switch between stream and parameters slews at PFA_ATT_SLEW until the command has
 * caught up, so a lost link or a late start does not step the attitude.
 */
matrix::Vector2f PFAPOSControl::desired_roll_pitch(const hrt_abstime now, const float dt)
{
	const float timeout = math::max(_param_att_tout.get(), 0.05f);
	debug_vect_s dv;

	if (_debug_vect_sub.update(&dv)
	    && (strncmp(dv.name, "att_cmd", sizeof(dv.name)) == 0)
	    && PX4_ISFINITE(dv.x) && PX4_ISFINITE(dv.y) && PX4_ISFINITE(dv.z)) {

		const Vector2f att(math::radians(math::constrain(dv.x, -90.f, 90.f)),
				   math::radians(math::constrain(dv.y, -90.f, 90.f)));
		const float dt_send = dv.z - _stream_t_send;	// NaN for the first sample
		const bool prev_fresh = (_stream_t_rx != 0) && ((dv.timestamp - _stream_t_rx) * 1e-6f < timeout);

		if (prev_fresh && (dt_send > 1e-3f) && (dt_send < timeout)) {
			_stream_rate = (att - _stream_att) / dt_send;

		} else {
			// first sample, restarted sender clock or a long gap: no rate information
			_stream_rate.setZero();
		}

		_stream_att = att;
		_stream_t_send = dv.z;
		_stream_t_rx = dv.timestamp;
	}

	const float age = (_stream_t_rx != 0 && now > _stream_t_rx) ? (now - _stream_t_rx) * 1e-6f : 0.f;
	const bool from_stream = (_param_att_src.get() == 1) && (_stream_t_rx != 0) && (age < timeout);

	Vector2f target(math::radians(_param_des_roll.get()), math::radians(_param_des_pitch.get()));

	if (from_stream) {
		target = _stream_att + _stream_rate * math::constrain(age, 0.f, _param_att_extr.get());
		target(0) = math::constrain(target(0), -M_PI_2_F, M_PI_2_F);
		target(1) = math::constrain(target(1), -M_PI_2_F, M_PI_2_F);
	}

	if (!_att_cmd_init) {
		// (re)start in flight: begin from the parameters so a stale stream sample is not stepped to
		_att_cmd_out = Vector2f(math::radians(_param_des_roll.get()), math::radians(_param_des_pitch.get()));
		_att_cmd_from_stream = false;
		_att_cmd_blending = true;
		_att_cmd_init = true;
	}

	if (from_stream != _att_cmd_from_stream) {
		_att_cmd_from_stream = from_stream;
		_att_cmd_blending = true;
	}

	if (_att_cmd_blending) {
		const float step = math::radians(math::max(_param_att_slew.get(), 1.f)) * dt;
		const Vector2f diff = target - _att_cmd_out;
		_att_cmd_out(0) += math::constrain(diff(0), -step, step);
		_att_cmd_out(1) += math::constrain(diff(1), -step, step);

		if (Vector2f(target - _att_cmd_out).norm() < math::radians(0.1f)) {
			_att_cmd_blending = false;
		}

	} else {
		_att_cmd_out = target;
	}

	return _att_cmd_out;
}

/**
 * Hold setpoint: latch position and heading when Hold is entered, follow EKF resets.
 */
void PFAPOSControl::update_hold_setpoint(const bool hold_mode, const vehicle_local_position_s &vlocal_pos)
{
	if (!hold_mode) {
		_hold_active = false;
		return;
	}

	if (!_hold_active) {
		_hold_setpoint = {};
		_hold_setpoint.position[0] = vlocal_pos.x;
		_hold_setpoint.position[1] = vlocal_pos.y;
		_hold_setpoint.position[2] = vlocal_pos.z;
		_hold_setpoint.yaw = Eulerf(Quatf(_vehicle_attitude.q))(2);
		_hold_setpoint.yawspeed = NAN;
		_hold_xy_reset_counter = vlocal_pos.xy_reset_counter;
		_hold_z_reset_counter = vlocal_pos.z_reset_counter;
		_hold_heading_reset_counter = vlocal_pos.heading_reset_counter;
		_hold_active = true;
		PX4_INFO("PFA hold at x %.2f y %.2f z %.2f yaw %.1f deg",
			 (double)vlocal_pos.x, (double)vlocal_pos.y, (double)vlocal_pos.z,
			 (double)math::degrees(_hold_setpoint.yaw));
	}

	// keep the latched setpoint at the same physical place across estimator resets
	if (vlocal_pos.xy_reset_counter != _hold_xy_reset_counter) {
		_hold_setpoint.position[0] += vlocal_pos.delta_xy[0];
		_hold_setpoint.position[1] += vlocal_pos.delta_xy[1];
		_hold_xy_reset_counter = vlocal_pos.xy_reset_counter;
	}

	if (vlocal_pos.z_reset_counter != _hold_z_reset_counter) {
		_hold_setpoint.position[2] += vlocal_pos.delta_z;
		_hold_z_reset_counter = vlocal_pos.z_reset_counter;
	}

	if (vlocal_pos.heading_reset_counter != _hold_heading_reset_counter) {
		_hold_setpoint.yaw = matrix::wrap_pi(_hold_setpoint.yaw + vlocal_pos.delta_heading);
		_hold_heading_reset_counter = vlocal_pos.heading_reset_counter;
	}

	_hold_setpoint.timestamp = vlocal_pos.timestamp;
}

void PFAPOSControl::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	_vcontrol_mode_sub.update(&_vcontrol_mode);


	/* update parameters from storage */
	parameters_update();

	//vehicle_attitude_s attitude;
	vehicle_local_position_s vlocal_pos;

	/* only run controller if local_pos changed */
	if (_vehicle_local_position_sub.update(&vlocal_pos)) {
		// calculate dt
		const float dt =
			math::constrain(((vlocal_pos.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vlocal_pos.timestamp_sample;
		_dt = dt;

		_vehicle_attitude_sub.update(&_vehicle_attitude);
		_trajectory_setpoint_sub.update(&_trajectory_setpoint);
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_status_sub.update(&_vehicle_status);

		_update_position_control_enable_time();

		// MANUAL / STABILIZED use the stick velocity controller below. POSCTL and
		// ALTCTL also have flag_control_manual_enabled set, but they are handled by
		// the position branch with the flight task's trajectory setpoint; running
		// both (as before) published two different attitude setpoints per cycle.
		const bool position_control_mode = _vcontrol_mode.flag_multicopter_position_control_enabled
						   || _vcontrol_mode.flag_control_offboard_enabled;

		if (_vcontrol_mode.flag_control_manual_enabled && !position_control_mode) {
			// manual mode: velocity control with level body (TVMD style)
			// Stick inputs control XY velocity, NOT roll/pitch attitude

			// Maximum XY velocity (m/s) - can be tuned via parameter
			const float max_vel_xy = 2.0f;
			const float max_yaw_rate = 2.0f;  // rad/s

			// Velocity control gain for manual mode
			// This compensates for the low PFA_GAIN_X_D/Y_D values (0.05) which are tuned for position control
			// For velocity control, we need a gain of ~1.0-2.0 to get responsive behavior
			const float vel_control_gain = 2.0f;

			// Convert stick to velocity commands (NED frame)
			// pitch stick forward (+1) = fly forward (positive X in NED)
			// roll stick right (+1) = fly right (positive Y in NED)
			// const float vel_x_cmd = _manual_control_setpoint.pitch * max_vel_xy;  // forward/backward
			// const float vel_y_cmd = _manual_control_setpoint.roll * max_vel_xy;   // left/right

			// Convert stick to velocity commands (heading-relative, then rotate to NED frame)
			// pitch stick forward (+1) = fly toward heading direction
			// roll stick right (+1) = fly 90° right of heading direction
			const float vel_x_body = _manual_control_setpoint.pitch * max_vel_xy;
			const float vel_y_body = _manual_control_setpoint.roll  * max_vel_xy;

			// Rotate body-frame velocity to NED world frame using current yaw
			const Quatf q_att_now(_vehicle_attitude.q);
			const Eulerf euler_att_now(q_att_now);
			const float yaw_now = euler_att_now(2);

			const float vel_x_cmd = cosf(yaw_now) * vel_x_body - sinf(yaw_now) * vel_y_body;
			const float vel_y_cmd = sinf(yaw_now) * vel_x_body + cosf(yaw_now) * vel_y_body;


			// Get current velocity from local position
			const float vel_x_cur = vlocal_pos.vx;
			const float vel_y_cur = vlocal_pos.vy;

			// Calculate velocity error and use as feedforward acceleration
			// This provides responsive velocity control without changing position control tuning
			const float acc_x = (vel_x_cmd - vel_x_cur) * vel_control_gain;
			const float acc_y = (vel_y_cmd - vel_y_cur) * vel_control_gain;

			// Yaw rate from yaw stick
			const float yaw_rate = _manual_control_setpoint.yaw * max_yaw_rate;

			// throttle normalize from -1~1 to 0~1
			const float level = (_manual_control_setpoint.throttle - (-1.0f)) / (1.0f - (-1.0f));

			// if throttle > 0.5, control is dominated by velocity command
			const float vel_z = - ((level < 0.5f) ? 0.0f : 2 * (level - 0.5f));

			// if throttle < 0.5, control is dominated by acceleration command (to resist gravity)
			// to avoid immediate rise of thrust setpoints
			const float acc_z = - CONSTANTS_ONE_G * ((level < 0.5f) ? (_manual_control_setpoint.throttle) : 0.0f);

			trajectory_setpoint_s traj_des{};
			traj_des.position[0] = NAN;
			traj_des.position[1] = NAN;
			traj_des.position[2] = NAN;
			traj_des.velocity[0] = vel_x_cmd;  // forward/backward from pitch stick
			traj_des.velocity[1] = vel_y_cmd;  // left/right from roll stick
			traj_des.velocity[2] = vel_z;
			traj_des.acceleration[0] = acc_x;  // feedforward acceleration for velocity control
			traj_des.acceleration[1] = acc_y;  // feedforward acceleration for velocity control
			traj_des.acceleration[2] = acc_z;

			_thrust_up_max = -1.0f;
			_thrust_xy_max = -0.3f * _thrust_up_max;

			// Keep body level (roll = pitch = 0), yaw controlled by stick
			// Extract current yaw from quaternion
			const Quatf q_att(_vehicle_attitude.q);
			const Eulerf euler_att(q_att);
			const float current_yaw = euler_att(2);  // yaw is the third element

			// Integrate yaw rate to get desired yaw
			// const float desired_yaw = current_yaw + yaw_rate * dt;
			// const Vector3f attitude_des = Vector3f(0.0f, 0.0f, desired_yaw);

			// Initialize desired yaw to current yaw on first entry to manual mode
			if (!_manual_yaw_initialized) {
			_manual_desired_yaw = current_yaw;
			_manual_yaw_initialized = true;
			}

			// Accumulate yaw rate from stick (persistent across iterations)
			_manual_desired_yaw += yaw_rate * dt;

			const Vector3f attitude_des = Vector3f(0.0f, 0.0f, _manual_desired_yaw);

			pose_controller_6dof(
				traj_des, attitude_des, _vehicle_attitude, vlocal_pos);
			// }


		// } else if ((_vcontrol_mode.flag_multicopter_position_control_enabled
		// 		|| _vcontrol_mode.flag_control_offboard_enabled)
		// 	&& (_trajectory_setpoint.timestamp >= _time_position_control_enabled)) {
		// 	// position mode / altitude mode

		} else {
		    _manual_yaw_initialized = false;  // 離開 manual mode 時重置，下次進入重新鎖定 yaw

		}

		// Hold (QGC "Hold" = AUTO_LOITER): keep the position and heading at the moment
		// the mode was entered. The setpoint is generated here, so the mode also works
		// without GPS (navigator / flight task need a global position for loiter).
		const bool hold_mode = position_control_mode
				       && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
		update_hold_setpoint(hold_mode, vlocal_pos);

		if (position_control_mode
		    && (hold_mode || (_trajectory_setpoint.timestamp >= _time_position_control_enabled))) {
			// position mode / altitude mode / offboard / auto modes
			const trajectory_setpoint_s &traj_sp = hold_mode ? _hold_setpoint : _trajectory_setpoint;

			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// TODO: the land process is not handled yet
			// Update takeoff state machine
			_takeoff.updateTakeoffState(
				_vcontrol_mode.flag_armed, false,
				((_param_takeoff_bypass.get() > 0) ? true : _vehicle_constraints.want_takeoff),
				_thrust_max, false, vlocal_pos.timestamp_sample);

			// Return the maximum thrust according to the takeoff state machine
			_thrust_up_max = _takeoff.updateRamp(dt, _thrust_max);
			// const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :

			const bool flying     = (_takeoff.getTakeoffState() >= TakeoffState::rampup);

			if (!flying) {
				_pos_yaw_valid = false;
				_att_cmd_init = false;
			}
			const bool ramping_up = (_takeoff.getTakeoffState() == TakeoffState::rampup);
			// const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

			// The flying state including the rampup phase
			if (flying) {
				// const Vector3f attitude_des = Vector3f(0.0f, 0.0f, _trajectory_setpoint.yaw); // roll and pitch = 0, yaw from trajectory setpoint
				// const Vector3f attitude_des = Vector3f(_manual_control_setpoint.roll, _manual_control_setpoint.pitch, _manual_control_setpoint.yaw); // roll and pitch from manual, yaw from trajectory setpoint
				const float yaw_sp = position_yaw_setpoint(traj_sp, dt);
				const Vector2f roll_pitch_des = desired_roll_pitch(hrt_absolute_time(), dt);
				const Vector3f attitude_des = Vector3f(roll_pitch_des(0), roll_pitch_des(1), yaw_sp);

				if (ramping_up) {
					// Reset position setpoint to current xy-position at the hovering height
					stabilization_controller_6dof(
						traj_sp, attitude_des, _vehicle_attitude, vlocal_pos);
				} else {
					pose_controller_6dof(
						traj_sp, attitude_des, _vehicle_attitude, vlocal_pos);
				}
			}
		} else {
			_pos_yaw_valid = false;

			// An update is necessary here because otherwise the takeoff state
			// doesn't get skipped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vcontrol_mode.flag_armed, false, false,
				_thrust_max, true, vlocal_pos.timestamp_sample);
		}
	}

	// Publish takeoff status
	// tilt_limit must be filled in: the POSCTL flight task (StickAccelerationXY) limits the
	// horizontal stick acceleration to tan(tilt_limit) * g. It was left at 0, so the stick
	// acceleration was scaled to zero and POSCTL did not move in x/y at all. The body stays
	// level here, so this only acts as a horizontal acceleration limit (MPC_TILTMAX_AIR).
	const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());
	const float tilt_limit = math::radians(_param_mpc_tiltmax_air.get());

	if (takeoff_state != _takeoff_status_pub.get().takeoff_state
	    || fabsf(tilt_limit - _takeoff_status_pub.get().tilt_limit) > FLT_EPSILON) {
		_takeoff_status_pub.get().takeoff_state = takeoff_state;
		_takeoff_status_pub.get().tilt_limit = tilt_limit;
		_takeoff_status_pub.get().timestamp = hrt_absolute_time();
		_takeoff_status_pub.update();
	}

	/* Only publish if any of the proper modes are enabled */
	if (_vcontrol_mode.flag_control_manual_enabled ||
	    _vcontrol_mode.flag_control_attitude_enabled) {
	}

	perf_end(_loop_perf);
}

int PFAPOSControl::task_spawn(int argc, char *argv[])
{
	PFAPOSControl *instance = new PFAPOSControl();

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

int PFAPOSControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int PFAPOSControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an partially fully actuated (PFA).
Publishes `attitude_setpoint` messages.
### Implementation
Currently, this implementation supports only a few modes:
 * Position mode with takeoff
 * Hold (AUTO_LOITER): holds the position / heading latched on entry, no GPS needed
 * Manual mode
### Examples
CLI usage example:
$ pfa_pos_control start
$ pfa_pos_control status
$ pfa_pos_control stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("pfa_pos_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start")
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int pfa_pos_control_main(int argc, char *argv[])
{
    return PFAPOSControl::main(argc, argv);
}
