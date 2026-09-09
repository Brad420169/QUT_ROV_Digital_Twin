"""
SIM-TO-REAL TUNING FILE
=======================

The ONLY file you should need to edit at the pool.

Everything in rov_config.py is the SIM value and stays canonical — the DT
is the reference. This file holds the corrections applied when running in
real mode, plus the handful of real-only hardware constants that change
between deployments (surface pressure, axis directions, RC range).

Nothing here affects sim mode except where noted. Sim always runs
unscaled.

Scaler convention:
    1.0  = transfers directly, no correction needed
    <1.0 = real vehicle is MORE responsive than the DT predicts
    >1.0 = real vehicle is LESS responsive than the DT predicts

Anything left at 1.0 after the trial is a DT fidelity result worth
reporting. Anything that is not 1.0 is a measured sim-to-real gap — keep
this block as the record of where sim and real diverge.

Edits take effect on the next teleop launch (values are resolved once at
node startup), so: pkill, edit, relaunch.
"""

# Command-side: flips the outgoing RC channel. Flip if the ROV moves
# the opposite way to the stick. Prefer fixing thruster direction in
# ArduSub (SERVOn_REVERSED) — use these only if QGC is already correct.
REAL_INVERT_SURGE_CMD = True
REAL_INVERT_HEAVE_CMD = False
REAL_INVERT_YAW_CMD   = False

# ─────────────────────────────────────────────────────────────────────
# 1. FIRST THINGS TO SET ON THE DAY
# ─────────────────────────────────────────────────────────────────────

# Bar30 reading with the ROV floating at the surface, in Pascals.
# Depth is computed relative to this, so if it is wrong every depth
# reading and the whole depth-hold loop is offset. Read it off the
# plotter or QGC before the ROV goes in and set it here.
REAL_SURFACE_PRESSURE_PA = 102137.0

# Water density (kg/m^3). 1031 is seawater; a chlorinated pool is much
# closer to fresh water — set 998.0 for a pool trial.
# NOTE: this one is shared with sim, so changing it changes both.
REAL_WATER_DENSITY = 1031.0
POOL_WATER_DENSITY = 998.0

# Axis direction sanity flags. Flip if a bench test shows the real
# vehicle rotating the opposite way to the DT for the same stick input.
REAL_YAW_INVERT = False
REAL_PITCH_INVERT = True


# ─────────────────────────────────────────────────────────────────────
# 2. MANUAL AUTHORITY — the safety knobs
# ─────────────────────────────────────────────────────────────────────
# Turn these DOWN first if the ROV feels twitchy or over-powered on the
# first wet run. Dropping all three to ~0.5 is a sensible first-splash
# setting, then work back up.

REAL_SCALE_MANUAL_SURGE = 1.0
REAL_SCALE_MANUAL_YAW   = 1.0
REAL_SCALE_MANUAL_HEAVE = 1.0


# ─────────────────────────────────────────────────────────────────────
# 3. DEPTH HOLD
# ─────────────────────────────────────────────────────────────────────
# Tuning order that works: get FF right first (neutral hover), then Kp,
# then Kd, then Ki last.

# Buoyancy trim feedforward. The DT models neither the foam nor the
# tether nose-up trim, so this is the least likely to stay at 1.0.
# Symptom -> action:
#   ROV sinks while holding depth      -> increase (more up-thrust)
#   ROV climbs while holding depth     -> decrease
REAL_SCALE_DEPTH_FF = 1.0

# Proportional. Sluggish to return to setpoint -> increase.
# Overshoots / hunts around the setpoint -> decrease.
REAL_SCALE_DEPTH_KP = 1.0

# Derivative. Increase to damp oscillation, but too much makes the
# vertical thrusters chatter on noisy Bar30 data.
REAL_SCALE_DEPTH_KD = 1.0

# Integral. Only needed if there is a persistent steady-state depth
# offset that FF has not removed. Raise last and slowly — this is the
# one that will wind up and surface the ROV if you get it wrong.
REAL_SCALE_DEPTH_KI = 1.0

# NOTE: scaling a gain changes loop dynamics, not just magnitude. A
# single shared factor is only strictly valid if the plant differs from
# the DT by a pure gain, which it will not. Expect to tune these three
# independently rather than moving them together.


# ─────────────────────────────────────────────────────────────────────
# 4. TRAJECTORY / HEADING HOLD
# ─────────────────────────────────────────────────────────────────────

# Forward speed used in trajectory mode.
REAL_SCALE_TRAJ_FORWARD = 0.5

# Scales the yaw output clamp in trajectory mode.
REAL_SCALE_TRAJ_YAW = 1.0

# Heading hold gains. The sim gains are currently 0.0, so these scalers
# do nothing until YAW_KP/KI/KD in rov_config.py are non-zero.
REAL_SCALE_YAW_KP = 1.0
REAL_SCALE_YAW_KI = 1.0
REAL_SCALE_YAW_KD = 1.0


# ─────────────────────────────────────────────────────────────────────
# 5. RC OUTPUT LIMITS — change only if you know why
# ─────────────────────────────────────────────────────────────────────
# ArduSub RC microseconds. Reducing RANGE caps thruster authority
# globally, below everything above — a blunt but effective safety limit
# for a first splash (e.g. 200 -> 1300-1700us, roughly half power).

REAL_RC_NEUTRAL_US = 1500   # stopped / neutral
REAL_RC_RANGE_US   = 400    # +-400us -> 1100-1900us full range


# ─────────────────────────────────────────────────────────────────────
# TRIAL LOG — write what you changed and why, straight into the file
# ─────────────────────────────────────────────────────────────────────
# date | param | old -> new | reason
# -----|-------|------------|-------
#
#
