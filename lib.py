from hub import port, motion_sensor, light_matrix
import runloop
import motor
import motor_pair
import math

# ===== Ports / pairing
PAIR = motor_pair.PAIR_1
motor_pair.pair(PAIR, port.A, port.B)

# ===== Polarity that behaved well on your robot
STEER_SIGN = 1

# ===== Heading (absolute); 0° after init
current_heading = 0.0

# ---------- helpers ----------
def wrap180(a): return (a + 180) % 360 - 180
def yaw_deg():return motion_sensor.tilt_angles()[0] / 10.0
def ang_diff(target, current): return (target - current + 540) % 360 - 180

# Encoders: flip A so forward counts positive
def left_deg():return -motor.relative_position(port.A)
def right_deg(): return motor.relative_position(port.B)
def avg_deg():return (left_deg() + right_deg()) / 2

WHEEL_DIAM_CM = 5.6
CIRCUM_CM    = math.pi * WHEEL_DIAM_CM
def cm_to_deg(cm: float) -> float: return (cm / CIRCUM_CM) * 360.0

# ---------- absolute-heading turn (NO yaw resets here) ----------
async def turn_to_heading(target_deg: float, k: float = 6.0, tol: float = 2.0):
    """
    Rotate in place until yaw ~= target_deg (absolute), P on speed.
    IMPORTANT: mapping makes +error => turn RIGHT on your robot.
    """
    in_tol_needed = 3
    in_tol = 0
    while True:
        err = ang_diff(target_deg, yaw_deg())# +err means we need to go RIGHT
        if abs(err) <= tol:
            in_tol += 1
            if in_tol >= in_tol_needed:
                break
        else:
            in_tol = 0

        vel = int(k * abs(err))
        if vel < 100: vel = 100
        if vel > 600: vel = 600

        # *** KEY FIX: flip mapping so +err => RIGHT with STEER_SIGN = -1 ***
        steer = STEER_SIGN * (-100 if err > 0 else +100)
        motor_pair.move(PAIR, steer, velocity=vel)
        await runloop.sleep_ms(20)

    motor_pair.stop(PAIR)

# User-facing wrappers (update global target; no resets)
async def turn_left(angle: float, k: float = 6.0, tol: float = 2.0):
    global current_heading
    target = wrap180(current_heading + angle)    # left = -angle
    await turn_to_heading(target, k=k, tol=tol)
    current_heading = target

async def turn_right(angle: float, k: float = 6.0, tol: float = 2.0):
    global current_heading
    target = wrap180(current_heading - angle)    # right = +angle
    await turn_to_heading(target, k=k, tol=tol)
    current_heading = target

# ---------- straight drive holding the global heading (NO resets) ----------
async def drive_cm_hold(distance_cm: float,
                        velocity: int = 320,
                        k_heading: float = 0.7,
                        steer_cap: int = 30,
                        loop_ms: int = 20):
    """
    Drive a distance while holding 'current_heading' with a P controller.
    """
    target_degs = abs(cm_to_deg(distance_cm))
    direction= 1 if distance_cm >= 0 else -1

    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.B, 0)

    while abs(avg_deg()) < target_degs:
        herr = ang_diff(current_heading, yaw_deg())# +herr => need RIGHT
        steer_p = int(k_heading * herr)
        if steer_p >steer_cap: steer_p =steer_cap
        if steer_p < -steer_cap: steer_p = -steer_cap

        # *** MATCH TURN FIX: flip mapping here too ***
        motor_pair.move(PAIR, STEER_SIGN * (-steer_p), velocity=abs(velocity) * direction)
        await runloop.sleep_ms(loop_ms)

    motor_pair.stop(PAIR)

# ---------- main ----------
async def main():
    global current_heading

    light_matrix.write("GO")

    # Initialize once: define forward as 0°
    motion_sensor.reset_yaw(0)
    current_heading = 0.0

    await drive_cm_hold(20, velocity=320, k_heading=3.7)
    await turn_right(90, k=6.0, tol=2.0)
    await drive_cm_hold(20, velocity=320, k_heading=3.7)
    await turn_left(90, k=6.0, tol=2.0)
    await drive_cm_hold(20, velocity=320, k_heading=3.7)

    light_matrix.write("OK")

runloop.run(main())
