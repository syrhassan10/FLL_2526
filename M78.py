from hub import port, motion_sensor
import runloop
import motor
import motor_pair
import math

# =========================
# DRIVEBASE / PORT SETUP
# =========================

LEFT_MOTOR_PORT = port.B# left wheel
RIGHT_MOTOR_PORT = port.F# right wheel

PAIR = motor_pair.PAIR_1
motor_pair.pair(PAIR, LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT)

# If robot steers the wrong way when correcting heading during drive, flip this
STEER_SIGN = -1# you said -1 works on your bot

# If robot turns the opposite way in place, flip this
TURN_DIR_SIGN = 1# normally 1, use -1 if right/left are reversed

# =========================
# GLOBAL STATE
# =========================

# Only kept so old code without hold_heading_deg still works.
# For your new style you should always pass heading explicitly.
current_heading = 0.0

# =========================
# GEOMETRY / TIMING / GAINS
# =========================

# Wheel geometry / distance calibration
WHEEL_DIAM_CM = 5.6
DIST_GAIN    = 1.00# <1.0 robot undertravels, >1.0 overtravels

def cm_to_deg(cm):
    """Convert chassis distance in cm to wheel rotation in degrees."""
    circ = math.pi * WHEEL_DIAM_CM
    return (cm * DIST_GAIN / circ) * 360.0

# Straight drive profile tuning
MAX_SPEED_STRAIGHT = 720    # deg/sec wheel max cruise
A_STRAIGHT        = 2200    # accel (deg/sec^2)
D_STRAIGHT        = 2600    # decel (deg/sec^2)
MIN_STRAIGHT    = 140    # don't go slower than this while moving

# Heading hold gains while driving straight
K_HEADING_P        = 0.85    # proportional on heading error
K_HEADING_D        = 0.10    # derivative on heading error
STEER_CAp        = 26    # typo-safe alias, just in case
STEER_CAP        = 26    # clamp steering command magnitude

# Turn profile tuning (in-place rotation)
TURN_KP            = 6.0    # proportional gain on turn distance
TURN_MIN_SPEED    = 90
TURN_MAX_SPEED    = 540
TURN_NEAR_DEG    = 30.0    # start slowing inside this error
TURN_FINAL_DEG    = 3.0    # creep zone
TURN_TOL_DEG    = 1.5    # consider done if |error| <= this
TURN_STABLE_LOOPS= 3    # stay in tol this many loops to finish

# Control loop timing
LOOP_MS = 10
DT    = LOOP_MS / 1000.0

# =========================
# SMALL MATH HELPERS
# =========================

def wrap180(a):
    """Wrap any angle to [-180, 180)."""
    return (a + 180) % 360 - 180

def ang_diff(target, current):
    """
    Signed shortest path (target - current) in degrees,
    guaranteed to be in [-180, 180).
    Positive result means target is clockwise from current.
    Negative means target is counterclockwise from current.
    """
    return (target - current + 540) % 360 - 180

def angle_diff_signed(target_deg, current_deg):
    # synonym for clarity in turn code
    return ang_diff(target_deg, current_deg)

# =========================
# SENSOR HELPERS
# =========================

def yaw_deg():
    """
    Get robot yaw heading in degrees in [-180, +180].
    You confirmed this is correct for your SPIKE:
    - Turn RIGHT (clockwise): goes 0 -> +179 -> wraps to -179 -> ... -> 0.
    We rely on calibrate_at_start() to zero this at the start of the run.
    """
    return motion_sensor.tilt_angles()[0] / -10.0

def left_deg():
    # left motor needs sign flip so forward counts positive
    return -motor.relative_position(LEFT_MOTOR_PORT)

def right_deg():
    return motor.relative_position(RIGHT_MOTOR_PORT)

def avg_wheel_deg():
    # average wheel rotation (deg). We count forward as positive.
    return (left_deg() + right_deg()) / 2.0

# =========================
# TURN HELPERS (LOW-LEVEL)
# =========================

def cw_error(target_deg, current_deg):
    """
    Clockwise-only remaining angle in [0..360).
    "How far do I still need to rotate CLOCKWISE from current to reach target?"
    """
    return (target_deg - current_deg) % 360

def ccw_error(target_deg, current_deg):
    """
    Counterclockwise-only remaining angle in [0..360).
    "How far do I still need to rotate COUNTERCLOCKWISE from current to reach target?"
    """
    return (current_deg - target_deg) % 360

async def _turn_step_until_done(target_angle_deg, spin_dir_sign):
    """
    Core rotation loop that ALWAYS spins a fixed direction:
    spin_dir_sign = +1 => clockwise (turn_right style)
    spin_dir_sign = -1 => counterclockwise (turn_left style)

    We ramp speed down as we get close, creep in final few degrees,
    and settle with tolerance.

    CRITICAL FIX:
    If we overshoot past the target in the chosen direction, we STOP.
    We do NOT try to wrap 360° around and "finish the rest".
    """

    stable_loops = 0

    while True:
        cur_yaw = yaw_deg()

        # signed shortest error in [-180, 180)
        signed_err = ang_diff(target_angle_deg, cur_yaw)
        # magnitude for speed calc
        mag = abs(signed_err)

        # --- overshoot guard ---
        # For clockwise turns:
        #signed_err > 0=> target is still clockwise ahead of us
        #signed_err <= 0 => we are at/past the target if we keep going clockwise
        #
        # For counterclockwise turns:
        #signed_err < 0=> target is still counterclockwise ahead
        #signed_err >= 0 => we are at/past target in that CCW direction
        if spin_dir_sign > 0:# clockwise enforced
            if signed_err <= 0:
                # we've arrived or overshot clockwise, stop driving and settle
                motor_pair.stop(PAIR)
                stable_loops += 1
                if stable_loops >= TURN_STABLE_LOOPS:
                    break
                await runloop.sleep_ms(LOOP_MS)
                continue
        else:# counterclockwise enforced
            if signed_err >= 0:
                motor_pair.stop(PAIR)
                stable_loops += 1
                if stable_loops >= TURN_STABLE_LOOPS:
                    break
                await runloop.sleep_ms(LOOP_MS)
                continue

        # if we're within tolerance (close enough), do settle logic too
        if mag <= TURN_TOL_DEG:
            motor_pair.stop(PAIR)
            stable_loops += 1
            if stable_loops >= TURN_STABLE_LOOPS:
                break
            await runloop.sleep_ms(LOOP_MS)
            continue
        else:
            stable_loops = 0

        # Base turn speed proportional to distance left
        raw_speed = TURN_KP * mag

        # Taper speed as we approach target
        if mag < TURN_NEAR_DEG:
            frac    = mag / TURN_NEAR_DEG    # 1 at far edge, 0 near goal
            near_cap = TURN_MIN_SPEED + frac * (TURN_MAX_SPEED - TURN_MIN_SPEED)
        else:
            near_cap = TURN_MAX_SPEED

        # Creep in last few degrees
        if mag < TURN_FINAL_DEG:
            near_cap = min(near_cap, 140)

        cmd_speed = min(raw_speed, near_cap, TURN_MAX_SPEED)
        cmd_speed = max(TURN_MIN_SPEED, cmd_speed)
        cmd_speed = int(cmd_speed)

        # Motor steer direction:
        #+100 => right motor fwd / left motor back (clockwise spin)
        #-100 => left motor fwd / right motor back (counterclockwise spin)
        steer_val = 100 if spin_dir_sign > 0 else -100
        steer_val *= TURN_DIR_SIGN# global flip in case wiring is mirrored

        motor_pair.move(PAIR, steer_val, velocity=cmd_speed)

        await runloop.sleep_ms(LOOP_MS)

    # final stop to be sure
    motor_pair.stop(PAIR)

# =========================
# PUBLIC TURN API
# =========================

async def turn_right(angle_abs_deg):
    """
    ABSOLUTE.
    "turn_right(45)" means:
    spin CLOCKWISE until robot faces absolute heading 45°.
    We DO NOT allow it to wrap all the way around if it overshoots.
    """
    target = wrap180(angle_abs_deg)
    await _turn_step_until_done(target_angle_deg=target, spin_dir_sign=+1)

    global current_heading
    current_heading = target

async def turn_left(angle_abs_deg):
    """
    ABSOLUTE.
    "turn_left(0)" means:
    spin COUNTERCLOCKWISE until robot faces absolute heading 0°.
    We DO NOT allow it to wrap all the way around if it overshoots.
    """
    target = wrap180(angle_abs_deg)
    await _turn_step_until_done(target_angle_deg=target, spin_dir_sign=-1)

    global current_heading
    current_heading = target

async def turn_to(angle_abs_deg):
    """
    Optional helper for 'shortest path'. Not required in your new style.
    """
    target = wrap180(angle_abs_deg)
    stable_loops = 0

    while True:
        cur = yaw_deg()
        err = angle_diff_signed(target, cur)# [-180,180)
        mag = abs(err)

        if mag <= TURN_TOL_DEG:
            motor_pair.stop(PAIR)
            stable_loops += 1
            if stable_loops >= TURN_STABLE_LOOPS:
                break
            await runloop.sleep_ms(LOOP_MS)
            continue
        else:
            stable_loops = 0

        raw_speed = TURN_KP * mag

        if mag < TURN_NEAR_DEG:
            frac    = mag / TURN_NEAR_DEG
            near_cap = TURN_MIN_SPEED + frac * (TURN_MAX_SPEED - TURN_MIN_SPEED)
        else:
            near_cap = TURN_MAX_SPEED

        if mag < TURN_FINAL_DEG:
            near_cap = min(near_cap, 140)

        cmd_speed = min(raw_speed, near_cap, TURN_MAX_SPEED)
        cmd_speed = max(TURN_MIN_SPEED, cmd_speed)
        cmd_speed = int(cmd_speed)

        spin_dir_sign = +1 if err > 0 else -1
        steer_val = 100 if spin_dir_sign > 0 else -100
        steer_val *= TURN_DIR_SIGN

        motor_pair.move(PAIR, steer_val, velocity=cmd_speed)
        await runloop.sleep_ms(LOOP_MS)

    motor_pair.stop(PAIR)

    global current_heading
    current_heading = target

# =========================
# DRIVE WITH HEADING HOLD
# (your PD + braking profile, with explicit heading)
# =========================

async def drive_cm_hold(distance_cm, speed=None, hold_heading_deg=None):
    """
    Drive a distance (cm) while holding a specific ABSOLUTE heading using PD steering
    and a trapezoidal speed profile. Positive distance_cm = forward, negative = backward.

    distance_cm: how far to drive (+ forward / - backward)
    speed: optional external cap on wheel speed (deg/sec). If None, uses MAX_SPEED_STRAIGHT.
    hold_heading_deg: absolute heading to lock onto (e.g. 0, 45, 90).
                    If None, we'll fall back to current_heading.
                    For new code, ALWAYS pass this.
    """

    global current_heading
    if hold_heading_deg is None:
        target_heading = current_heading
    else:
        target_heading = wrap180(hold_heading_deg)

    # total travel in wheel degrees
    target_degs = abs(cm_to_deg(distance_cm))

    # +1 forward, -1 backward
    direction = 1 if distance_cm >= 0 else -1

    # max allowable wheel speed
    v_cap = min(
        abs(speed) if speed is not None else MAX_SPEED_STRAIGHT,
        MAX_SPEED_STRAIGHT
    )

    # zero wheel encoders for this segment
    motor.reset_relative_position(LEFT_MOTOR_PORT, 0)
    motor.reset_relative_position(RIGHT_MOTOR_PORT, 0)

    # state for speed ramp and D term
    v = 0.0
    prev_err = ang_diff(target_heading, yaw_deg())

    while True:
        progressed = abs(avg_wheel_deg())
        remaining= max(target_degs - progressed, 0.0)

        # done?
        if remaining <= 1.0:
            break

        # --- PD steering on heading ---
        herr = ang_diff(target_heading, yaw_deg())# proportional
        derr = (herr - prev_err) / DT            # derivative
        prev_err = herr

        steer_raw = K_HEADING_P * herr + K_HEADING_D * derr

        # clamp steering
        if steer_raw >STEER_CAP: steer_raw =STEER_CAP
        if steer_raw < -STEER_CAP: steer_raw = -STEER_CAP

        steer_cmd = int(steer_raw)

        # adapt steering for motor_pair and forward/back
        steer_cmd = STEER_SIGN * (-steer_cmd) * direction

        # --- trapezoid speed profile with braking distance cap ---
        v_brake= math.sqrt(max(0.0, 2.0 * D_STRAIGHT * remaining))

        v_target = v_cap
        v_target = min(v_target, v_brake)
        v_target = max(min(v_target, v_cap), MIN_STRAIGHT)

        # accel/decel toward v_target
        if v < v_target:
            v = min(v + A_STRAIGHT * DT, v_target)
        else:
            v = max(v - D_STRAIGHT * DT, v_target)

        motor_pair.move(PAIR, steer_cmd, velocity=int(v) * direction)

        await runloop.sleep_ms(LOOP_MS)

    motor_pair.stop(PAIR)

# =========================
# CALIBRATION AT START
# =========================

def calibrate_at_start():
    """
    Call this ONCE at the start of the run in base.
    - Zero yaw so the robot's launch direction becomes 0°.
    - Zero both drive encoders.
    - Reset current_heading to 0 so absolute angles line up with field.
    """
    motion_sensor.reset_yaw(0)
    motor.reset_relative_position(LEFT_MOTOR_PORT, 0)
    motor.reset_relative_position(RIGHT_MOTOR_PORT, 0)

    global current_heading
    current_heading = 0.0

# =========================
# EXAMPLE MAIN
# =========================

async def main():
    calibrate_at_start()



    # Your desired calling style with absolute headings:
    # await drive_cm_hold(7, 700, 0)

    # await turn_right(45)
    # await drive_cm_hold(23, 700, 45)
    # await drive_cm_hold(-10, 700, 45)

    # await turn_left(0)



    await drive_cm_hold(60, 700, 0)
    await turn_right(90)
    motor.run_for_degrees(port.C, -370, 650)
    await drive_cm_hold(15, 700, 90)
    await turn_right(124)
    await drive_cm_hold(13, 300, 124)
    motor.run_for_degrees(port.C, 370, 550)
    await drive_cm_hold(-10, 400, 124)
    await turn_left(90)


    #2nd mission
    await drive_cm_hold(55, 700, 90)
    await turn_right(122)
    await motor.run_for_degrees(port.C, -370, 650)
    motor.run_for_degrees(port.C, 370, 650)
    await turn_left(90)

    #3rd mission
    # await drive_cm_hold(25, 700, 90)
    # await turn_right(135)
    # await drive_cm_hold(14, 500, 135)
    # # await drive_cm_hold(-7, 700, 140)
    # await drive_cm_hold(-3, 700, 135)
    # await turn_left(90)

    #4th and 5th mission
    await drive_cm_hold(44, 700, 90)
    motor.run_for_degrees(port.E, -120, 650)
    await turn_left(45)
    motor.run_for_degrees(port.E, 100, 650)
    await turn_right(87)
    await drive_cm_hold(8, 700, 87)

    await turn_right(180)
    await drive_cm_hold(90, 1000, 180)


    return

runloop.run(main())
