/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file pfa_pos_control_params.c
 *
 * Parameters defined by the position control task for partially fully actuated (PFA) systems.
 *
 * This module is a modification of the uuv attitide control module and is designed for
 * partially fully actuated systems.
 *
 * @author Yen-Cheng Chu <sciyen.ycc@gmail.com>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */
/**
 * Gain of P controller X
 *
 * @group PFA Position Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_GAIN_X_P, 0.05f);
/**
 * Gain of P controller Y
 *
 * @group PFA Position Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_GAIN_Y_P, 0.05f);
/**
 * Gain of P controller Z
 *
 * @group PFA Position Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_GAIN_Z_P, 0.50f);

/**
 * Gain of D controller X
 *
 * @group PFA Position Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_GAIN_X_D, 0.05f);

/**
 * Gain of D controller Y
 *
 * @group PFA Position Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_GAIN_Y_D, 0.05f);

/**
 * Gain of D controller Z
 *
 * @group PFA Position Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_GAIN_Z_D, 2.00f);

/**
 * Mass of the vehicle in Kg
 *
 * @group PFA Position Control
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(PFA_VEH_MASS, 1.400f);

/**
 * Maximum thrust of the vechicle in Newton
 *
 * @group PFA Position Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(PFA_MAX_THR, 24.00f);

/**
 * Desired roll angle for position control mode (degrees)
 *
 * Sets the desired body roll angle passed to pose_controller_6dof as
 * euler_attitude_des(0).  Positive = right-wing down.
 *
 * @min -45.0
 * @max  45.0
 * @unit deg
 * @group PFA Position Control
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(PFA_DES_ROLL, 0.0f);

/**
 * Desired pitch angle for position control mode (degrees)
 *
 * Sets the desired body pitch angle passed to pose_controller_6dof as
 * euler_attitude_des(1).  Positive = nose up.
 * The position controller still computes the thrust in NED using current
 * attitude feedback; this parameter only changes the attitude target that
 * is forwarded to the attitude controller.
 *
 * @min -45.0
 * @max  45.0
 * @unit deg
 * @group PFA Position Control
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(PFA_DES_PITCH, 0.0f);

/**
 * Bypassing takeoff process (takeoff immediately after arming)
 *
 * @group PFA Position Control
 */
PARAM_DEFINE_INT32(PFA_TKF_BYP, 1);

/**
 * Vertical priority when the body x/y thrust is saturated
 *
 * pose_controller_6dof limits the body-frame thrust per axis (x/y to
 * +-0.3). When the vehicle is tilted, part of the world-frame lift comes
 * from the body x/y components, so clipping them makes the vehicle sink.
 * If enabled, the body z thrust is re-computed after the x/y clipping so
 * that the world-frame vertical force still matches the demand; the
 * horizontal shortfall is accepted instead (position drifts, altitude is kept).
 * Only active while x/y are clipped and the tilt is below 60 deg.
 *
 * @boolean
 * @group PFA Position Control
 */
PARAM_DEFINE_INT32(PFA_VERT_PRIO, 1);

/**
 * Minimum body lift while the body x/y thrust is saturated
 *
 * Fraction of the hover thrust (PFA_VEH_MASS * g / PFA_MAX_THR) that the
 * body z thrust may not drop below while x/y are clipped. Last line of
 * defence against large transient horizontal demands. Not applied when
 * x/y are not clipped, so normal descents and landing are unaffected, and
 * never above the takeoff ramp limit. 0 disables the floor.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group PFA Position Control
 */
PARAM_DEFINE_FLOAT(PFA_FUP_MIN, 0.6f);

/**
 * Attitude yield when the desired force leaves the thrust cone
 *
 * The body can only tilt its thrust by the servo range (CA_SV_TL0/1_MAXA)
 * relative to the body z axis. When the commanded attitude (PFA_DES_ROLL /
 * PFA_DES_PITCH, or level) would need a body-frame horizontal force outside
 * the cone of half angle PFA_YIELD_TILT around the thrust axis (judged with the
 * nominal force: gravity + acceleration feedforward), the attitude setpoint is
 * rotated towards that force (closed form, smallest rotation, yaw kept) until
 * it lies on the cone boundary. If the vehicle is nevertheless pushed away
 * (low-passed full force more than the servo range + 15 deg off), it leans
 * further into the force, up to MPC_TILTMAX_AIR. Position is
 * then kept at the cost of not reaching the commanded tilt. While enabled the
 * body x/y thrust is also limited by a cone, |b_xy| <= tan(servo max) * |b_z|,
 * instead of only the fixed +-0.3 box.
 *
 * @boolean
 * @group PFA Position Control
 */
PARAM_DEFINE_INT32(PFA_ATT_YIELD, 1);

/**
 * Thrust cone half angle for the attitude yield
 *
 * Largest angle allowed between the nominal force (gravity + acceleration
 * feedforward) and the thrust axis of the commanded attitude. Keep it below the
 * servo range (CA_SV_TL*_MAXA, 30 deg) by more than the steady attitude
 * tracking error (about 5-7 deg without an attitude integrator), so that the
 * actual attitude stays inside the servo range and the servos keep some range
 * for the attitude torques.
 *
 * @unit deg
 * @min 5.0
 * @max 40.0
 * @decimal 1
 * @increment 1.0
 * @group PFA Position Control
 */
PARAM_DEFINE_FLOAT(PFA_YIELD_TILT, 20.0f);

/**
 * Recovery rate of the attitude yield (drift stage)
 *
 * Rate at which the drift-stage yield (vehicle pushed away, force beyond the
 * servo range + 15 deg) shrinks again, so the attitude setpoint returns
 * smoothly. Growing is immediate. The feasibility stage (commanded tilt outside
 * the PFA_YIELD_TILT cone) is not rate limited: it follows the command.
 *
 * @unit deg/s
 * @min 5.0
 * @max 360.0
 * @decimal 1
 * @increment 5.0
 * @group PFA Position Control
 */
PARAM_DEFINE_FLOAT(PFA_YIELD_RATE, 45.0f);

/**
 * Source of the desired roll and pitch
 *
 * 0: PFA_DES_ROLL / PFA_DES_PITCH.
 * 1: MAVLink DEBUG_VECT named "att_cmd" (x = roll deg, y = pitch deg, z = sender
 * time of the sample in s), extrapolated with the rate of the last two samples.
 * Falls back to PFA_DES_ROLL / PFA_DES_PITCH when no sample arrived for
 * PFA_ATT_TOUT. The attitude yield and all force limits apply to both sources.
 *
 * @value 0 Parameters
 * @value 1 DEBUG_VECT stream
 * @group PFA Position Control
 */
PARAM_DEFINE_INT32(PFA_ATT_SRC, 0);

/**
 * Timeout of the streamed attitude command
 *
 * Without a new DEBUG_VECT "att_cmd" sample for this time, the desired roll and
 * pitch slew back to PFA_DES_ROLL / PFA_DES_PITCH at PFA_ATT_SLEW.
 *
 * @unit s
 * @min 0.05
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group PFA Position Control
 */
PARAM_DEFINE_FLOAT(PFA_ATT_TOUT, 0.5f);

/**
 * Extrapolation horizon of the streamed attitude command
 *
 * Between samples the command continues with the rate of the last two samples
 * for at most this time, then holds. 0 disables the extrapolation (sample and
 * hold).
 *
 * @unit s
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group PFA Position Control
 */
PARAM_DEFINE_FLOAT(PFA_ATT_EXTR, 0.3f);

/**
 * Slew rate when switching the attitude command source
 *
 * Applied on every switch between the DEBUG_VECT stream and the parameters
 * (stream start, timeout, PFA_ATT_SRC change) until the command has caught up.
 * Must be larger than the peak rate of the streamed waveform, otherwise the
 * command only catches up at a turning point.
 *
 * @unit deg/s
 * @min 5.0
 * @max 360.0
 * @decimal 1
 * @increment 5.0
 * @group PFA Position Control
 */
PARAM_DEFINE_FLOAT(PFA_ATT_SLEW, 60.0f);

/**
 * Position setpoint extrapolation limit
 *
 * Offboard setpoints arrive at a few Hz over a telemetry link. Between two
 * samples the position and velocity targets are carried forward with the
 * sample's velocity and acceleration (p + v*age + a*age^2/2, v + a*age) for
 * at most this time, then held.
 * Removes the position staircase that makes the vehicle surge and brake at
 * the setpoint rate when it tracks a moving reference. 0 disables it (sample
 * and hold). Not applied in Hold mode.
 *
 * @unit s
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group PFA Position Control
 */
PARAM_DEFINE_FLOAT(PFA_SP_EXTR, 0.3f);
