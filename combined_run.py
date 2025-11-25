"""
LEGO Spike Prime - Sequential Runs
Press LEFT then RIGHT to execute each run in sequence
"""

from hub import port, motion_sensor, light_matrix, button
import runloop
import motor
import motor_pair
import math


class RobotConfig:
    LEFT_MOTOR = port.A
    RIGHT_MOTOR = port.F
    MOTOR_PAIR_ID = motor_pair.PAIR_1
    WHEEL_DIAMETER_CM = 5.6
    DISTANCE_GAIN = 1.00
    STEER_SIGN = -1
    TURN_DIRECTION_SIGN = 1
    MAX_SPEED_STRAIGHT = 720
    ACCEL_STRAIGHT = 2200
    DECEL_STRAIGHT = 2600
    MIN_SPEED_STRAIGHT = 140
    KP_HEADING = 0.85
    KD_HEADING = 0.10
    STEER_LIMIT = 26
    TURN_KP = 6.0
    TURN_MIN_SPEED = 90
    TURN_MAX_SPEED = 540
    TURN_NEAR_THRESHOLD = 30.0
    TURN_FINAL_THRESHOLD = 3.0
    TURN_TOLERANCE = 1.5
    TURN_STABLE_COUNT = 3
    LOOP_PERIOD_MS = 10
    LOOP_PERIOD_SEC = LOOP_PERIOD_MS / 1000.0


class RobotState:
    def __init__(self):
        self.current_heading = 0.0
        self.is_calibrated = False

    def set_heading(self, heading_deg):
        self.current_heading = wrap_angle(heading_deg)

    def get_heading(self):
        return self.current_heading


_state = RobotState()


def wrap_angle(angle_deg):
    return (angle_deg + 180) % 360 - 180


def angle_difference(target_deg, current_deg):
    return wrap_angle(target_deg - current_deg)


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


class Sensors:
    @staticmethod
    def yaw_deg():
        return motion_sensor.tilt_angles()[0] / -10.0

    @staticmethod
    def left_wheel_deg():
        return -motor.relative_position(RobotConfig.LEFT_MOTOR)

    @staticmethod
    def right_wheel_deg():
        return motor.relative_position(RobotConfig.RIGHT_MOTOR)

    @staticmethod
    def average_wheel_deg():
        return (Sensors.left_wheel_deg() + Sensors.right_wheel_deg()) / 2.0

    @staticmethod
    def reset_wheels():
        motor.reset_relative_position(RobotConfig.LEFT_MOTOR, 0)
        motor.reset_relative_position(RobotConfig.RIGHT_MOTOR, 0)

    @staticmethod
    def reset_yaw():
        motion_sensor.reset_yaw(0)


def cm_to_wheel_degrees(distance_cm):
    circumference = math.pi * RobotConfig.WHEEL_DIAMETER_CM
    return (distance_cm * RobotConfig.DISTANCE_GAIN / circumference) * 360.0


class MotionController:
    @staticmethod
    async def _turn_with_direction(target_deg, spin_direction):
        target = wrap_angle(target_deg)
        stable_count = 0

        while True:
            current = Sensors.yaw_deg()
            signed_error = angle_difference(target, current)
            error_magnitude = abs(signed_error)

            if spin_direction > 0:
                if signed_error <= 0:
                    motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)
                    stable_count += 1
                    if stable_count >= RobotConfig.TURN_STABLE_COUNT:
                        break
                    await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)
                    continue
            else:
                if signed_error >= 0:
                    motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)
                    stable_count += 1
                    if stable_count >= RobotConfig.TURN_STABLE_COUNT:
                        break
                    await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)
                    continue

            if error_magnitude <= RobotConfig.TURN_TOLERANCE:
                motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)
                stable_count += 1
                if stable_count >= RobotConfig.TURN_STABLE_COUNT:
                    break
                await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)
                continue
            else:
                stable_count = 0

            raw_speed = RobotConfig.TURN_KP * error_magnitude

            if error_magnitude < RobotConfig.TURN_NEAR_THRESHOLD:
                taper = error_magnitude / RobotConfig.TURN_NEAR_THRESHOLD
                speed_cap = (RobotConfig.TURN_MIN_SPEED +
                        taper * (RobotConfig.TURN_MAX_SPEED - RobotConfig.TURN_MIN_SPEED))
            else:
                speed_cap = RobotConfig.TURN_MAX_SPEED

            if error_magnitude < RobotConfig.TURN_FINAL_THRESHOLD:
                speed_cap = min(speed_cap, 140)

            command_speed = clamp(raw_speed,
                                RobotConfig.TURN_MIN_SPEED,
                                min(speed_cap, RobotConfig.TURN_MAX_SPEED))
            command_speed = int(command_speed)

            steer_value = 100 if spin_direction > 0 else -100
            steer_value *= RobotConfig.TURN_DIRECTION_SIGN

            motor_pair.move(RobotConfig.MOTOR_PAIR_ID, steer_value, velocity=command_speed)
            await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)

        motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)

    @staticmethod
    async def _drive_with_pd_steering(distance_cm, max_speed, target_heading):
        target_wheel_deg = cm_to_wheel_degrees(abs(distance_cm))
        direction = 1 if distance_cm > 0 else -1
        target_heading = wrap_angle(target_heading)

        Sensors.reset_wheels()

        current_velocity = RobotConfig.MIN_SPEED_STRAIGHT
        prev_heading_error = 0.0

        while True:
            traveled = abs(Sensors.average_wheel_deg())
            remaining = target_wheel_deg - traveled

            if remaining <= 0:
                break

            current_heading = Sensors.yaw_deg()
            heading_error = angle_difference(target_heading, current_heading)
            heading_derivative = heading_error - prev_heading_error
            prev_heading_error = heading_error

            steering_raw = (RobotConfig.KP_HEADING * heading_error +
                        RobotConfig.KD_HEADING * heading_derivative)

            steering_command = clamp(steering_raw,
                                -RobotConfig.STEER_LIMIT,
                                RobotConfig.STEER_LIMIT)

            steering_command = (RobotConfig.STEER_SIGN *
                            (-steering_command) * direction)
            steering_command = int(steering_command)

            braking_velocity = math.sqrt(max(0.0,
                2.0 * RobotConfig.DECEL_STRAIGHT * remaining))

            target_velocity = min(max_speed, braking_velocity)
            target_velocity = max(target_velocity, RobotConfig.MIN_SPEED_STRAIGHT)

            if current_velocity < target_velocity:
                current_velocity = min(
                    current_velocity + RobotConfig.ACCEL_STRAIGHT * RobotConfig.LOOP_PERIOD_SEC,
                    target_velocity
                )
            else:
                current_velocity = max(
                    current_velocity - RobotConfig.DECEL_STRAIGHT * RobotConfig.LOOP_PERIOD_SEC,
                    target_velocity
                )

            motor_pair.move(RobotConfig.MOTOR_PAIR_ID,
                        steering_command,
                        velocity=int(current_velocity) * direction)

            await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)

        motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)


def calibrate():
    # Only pair motors if not already paired
    if not _state.is_calibrated:
        motor_pair.pair(RobotConfig.MOTOR_PAIR_ID,
                        RobotConfig.LEFT_MOTOR,
                        RobotConfig.RIGHT_MOTOR)

    # Always reset sensors
    Sensors.reset_yaw()
    Sensors.reset_wheels()
    _state.set_heading(0.0)
    _state.is_calibrated = True


async def turn_right(target_heading_deg):
    target = wrap_angle(target_heading_deg)
    await MotionController._turn_with_direction(target, spin_direction=+1)
    _state.set_heading(target)


async def turn_left(target_heading_deg):
    target = wrap_angle(target_heading_deg)
    await MotionController._turn_with_direction(target, spin_direction=-1)
    _state.set_heading(target)


async def turn_to(target_heading_deg):
    target = wrap_angle(target_heading_deg)
    stable_count = 0

    while True:
        current = Sensors.yaw_deg()
        error = angle_difference(target, current)
        error_magnitude = abs(error)

        if error_magnitude <= RobotConfig.TURN_TOLERANCE:
            motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)
            stable_count += 1
            if stable_count >= RobotConfig.TURN_STABLE_COUNT:
                break
            await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)
            continue
        else:
            stable_count = 0

        raw_speed = RobotConfig.TURN_KP * error_magnitude

        if error_magnitude < RobotConfig.TURN_NEAR_THRESHOLD:
            taper = error_magnitude / RobotConfig.TURN_NEAR_THRESHOLD
            speed_cap = (RobotConfig.TURN_MIN_SPEED +
                    taper * (RobotConfig.TURN_MAX_SPEED - RobotConfig.TURN_MIN_SPEED))
        else:
            speed_cap = RobotConfig.TURN_MAX_SPEED

        if error_magnitude < RobotConfig.TURN_FINAL_THRESHOLD:
            speed_cap = min(speed_cap, 140)

        command_speed = clamp(raw_speed,
                            RobotConfig.TURN_MIN_SPEED,
                            min(speed_cap, RobotConfig.TURN_MAX_SPEED))
        command_speed = int(command_speed)

        spin_direction = 1 if error > 0 else -1
        steer_value = 100 if spin_direction > 0 else -100
        steer_value *= RobotConfig.TURN_DIRECTION_SIGN

        motor_pair.move(RobotConfig.MOTOR_PAIR_ID, steer_value, velocity=command_speed)
        await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)

    motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)
    _state.set_heading(target)


async def drive_cm(distance_cm, speed=None, heading=None):
    if speed is None:
        speed = RobotConfig.MAX_SPEED_STRAIGHT

    if heading is None:
        target_heading = _state.get_heading()
    else:
        target_heading = wrap_angle(heading)

    await MotionController._drive_with_pd_steering(distance_cm, speed, target_heading)


# ===== RUNS =====

async def run_1():
    calibrate()
    await drive_cm(68)
    await runloop.sleep_ms(100)
    await turn_left(-30)
    await drive_cm(3)

    await motor.run_for_degrees(port.E, 280, 720)
    await drive_cm(7)
    await motor.run_for_degrees(port.E, -95, 720)
    motor_pair.move_for_degrees(RobotConfig.MOTOR_PAIR_ID, -1700, -10, velocity=900, acceleration=9000)
    await runloop.sleep_ms(300)
    motor.run_for_degrees(port.C, -69, 1150)


async def run_2():
    calibrate()
    await drive_cm(30, 500, 0)
    await motor.run_for_degrees(port.E, 260, 720)
    await drive_cm(-15, 300, 0)
    motor.run_for_degrees(port.E, -260, 720)
    await turn_left(-45)
    await drive_cm(13, 300, -45)
    await turn_right(0)
    await drive_cm(27, 1000, 0)


async def run_3():
    calibrate()
    await drive_cm(-68)
    await turn_left(-87.5)
    await drive_cm(17.5, 200)
    await motor.run_for_degrees(port.C, 170, 100)
    await motor.run_for_degrees(port.E, 25, 300)
    await runloop.sleep_ms(180)
    await drive_cm(-12.5, 200)
    await turn_right(0)
    motor_pair.move_for_degrees(RobotConfig.MOTOR_PAIR_ID, 1700, 2, velocity=1000, acceleration=10000)


async def run_4():
    calibrate()
    await drive_cm(60, 500, 0)
    await turn_right(90)
    motor.run_for_degrees(port.C, -370, 650)
    await drive_cm(14.5, 500, 90)
    await turn_right(122)
    await drive_cm(13.5, 300, 124)
    motor.run_for_degrees(port.C, 370, 550)
    await drive_cm(-12, 400, 124)
    await turn_left(90)
    await drive_cm(60, 700, 90)
    await turn_right(125)
    await motor.run_for_degrees(port.C, -370, 650)
    motor.run_for_degrees(port.C, 370, 650)
    await turn_left(90)
    await drive_cm(25, 700, 90)
    await turn_right(135)
    await drive_cm(15, 500, 135)
    await drive_cm(-3, 700, 135)
    await turn_left(90)
    await drive_cm(20, 700, 90)
    await turn_right(180)
    motor_pair.move_for_degrees(RobotConfig.MOTOR_PAIR_ID, 1700, 0, velocity=1100, acceleration=10000)


async def run_5():
    calibrate()
    await drive_cm(30,500)
    for _ in range(3):
        await motor.run_for_degrees(port.E, 150, 600)
        await motor.run_for_degrees(port.E, -150, 600)

    await turn_left(-27)
    await drive_cm(33,500)
    await turn_right(47)
    await drive_cm(12,300)
    await motor.run_for_degrees(port.C, -155, 200)
    await drive_cm(5,300)
    await motor.run_for_degrees(port.C, 130, 200)
    await drive_cm(-4.5,50)
    await turn_right(67)
    await drive_cm(-3,50)
    await turn_right(90)
    await drive_cm(-10,300)
    await turn_left(47)
    await drive_cm(7.5,300)
    await turn_left(15)
    await drive_cm(-15,500)
    await turn_left(-15)
    await drive_cm(-70,720)

async def run_6():
    calibrate()
    await turn_right(48)
    motor.run_for_degrees(port.E, 300, 720)
    await drive_cm(34, 500)
    await turn_left(10)
    await drive_cm(-5, 500)
    await turn_left(-30)
    motor.run_for_degrees(port.E, -20, 720)
    await drive_cm(30, 500)
    await turn_right(65)
    await drive_cm(8, 500)
    await motor.run_for_degrees(port.E, 100, 720)
    await drive_cm(-5, 500)
    await turn_left(0)
    await drive_cm(30, 500)
    await turn_right(45)
    await drive_cm(17, 500)
    await motor.run_for_degrees(port.E, -200, 720)
    await drive_cm(-5, 500)


# ===== SEQUENTIAL CONTROL =====

async def wait_for_left_then_right():
    """Wait for user to press LEFT then RIGHT"""
    # Clear any previous presses
    button.pressed(button.LEFT)
    button.pressed(button.RIGHT)

    # Wait for LEFT press
    left_pressed = False
    while not left_pressed:
        if button.pressed(button.LEFT) > 0:
            left_pressed = True
            light_matrix.write("L")
            await runloop.sleep_ms(300)
        await runloop.sleep_ms(50)

    # Now wait for RIGHT press
    while True:
        if button.pressed(button.RIGHT) > 0:
            light_matrix.write("R")
            await runloop.sleep_ms(300)
            light_matrix.clear()
            return
        await runloop.sleep_ms(50)


async def main():
    """
    Sequential runs: 1->2->3->4->5
    Between each run: Press LEFT then RIGHT to continue
    """
    runs = [run_1, run_2, run_3, run_4, run_5, run_6]

    for i, run_func in enumerate(runs):
        run_number = i + 1

        # Show which run is next
        light_matrix.write(str(run_number))

        # Wait for LEFT then RIGHT
        await wait_for_left_then_right()

        # Execute the run
        await run_func()

        # Brief pause after completion
        await runloop.sleep_ms(500)

    # All runs complete
    light_matrix.write("✓")


runloop.run(main())