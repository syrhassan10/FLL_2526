"""
LEGO Spike Prime Robot Library - Fixed Version
===============================================
A comprehensive library for precision robot navigation with gyro-based heading control,
trapezoidal motion profiles, and PD steering.

Key Features:
- Absolute heading-based navigation
- PD steering for straight-line accuracy
- Trapezoidal velocity profiles with dynamic braking
- Overshoot prevention in turns
- Modular, well-documented API

Author: Enhanced from original version
"""

from hub import port, motion_sensor
import runloop
import motor
import motor_pair
import math


# =========================
# CONFIGURATION CLASS
# =========================

class RobotConfig:
    """
    Central configuration for all robot parameters.
    Modify these values to tune your robot's behavior.
    """

    # Motor Ports
    LEFT_MOTOR = port.A
    RIGHT_MOTOR = port.F
    MOTOR_PAIR_ID = motor_pair.PAIR_1

    # Hardware Calibration
    WHEEL_DIAMETER_CM = 5.6
    DISTANCE_GAIN = 1.00# Calibration factor for distance accuracy

    # Sign Conventions (flip if robot behaves incorrectly)
    STEER_SIGN = -1# Flip if steering correction goes wrong way
    TURN_DIRECTION_SIGN = 1# Flip if in-place turns go wrong way

    # Straight Drive Motion Profile
    MAX_SPEED_STRAIGHT = 720# deg/sec - maximum wheel speed
    ACCEL_STRAIGHT = 2200# deg/sec² - acceleration rate
    DECEL_STRAIGHT = 2600# deg/sec² - deceleration rate
    MIN_SPEED_STRAIGHT = 140# deg/sec - minimum speed while moving

    # PD Steering Gains (for heading hold during straight drives)
    KP_HEADING = 0.85# Proportional gain
    KD_HEADING = 0.10# Derivative gain
    STEER_LIMIT = 26# Maximum steering correction magnitude

    # Turn-in-Place Profile
    TURN_KP = 6.0# Proportional gain for turn speed
    TURN_MIN_SPEED = 90# deg/sec
    TURN_MAX_SPEED = 140# deg/sec
    TURN_NEAR_THRESHOLD = 30.0# degrees - start slowing down
    TURN_FINAL_THRESHOLD = 3.0# degrees - creep zone
    TURN_TOLERANCE = 1.5# degrees - consider "done"
    TURN_STABLE_COUNT = 3# loops to stay in tolerance before finishing

    # Control Loop Timing
    LOOP_PERIOD_MS = 10
    LOOP_PERIOD_SEC = LOOP_PERIOD_MS / 1000.0


# =========================
# ROBOT STATE
# =========================

class RobotState:
    """Tracks the robot's current state."""

    def __init__(self):
        self.current_heading = 0.0# Absolute heading in degrees
        self.is_calibrated = False

    def set_heading(self, heading_deg):
        """Update the current heading."""
        self.current_heading = wrap_angle(heading_deg)

    def get_heading(self):
        """Get the current heading."""
        return self.current_heading


# Global state instance
_state = RobotState()


# =========================
# MATH UTILITIES
# =========================

def wrap_angle(angle_deg):
    """
    Wrap any angle to the range [-180, 180).

    Args:
        angle_deg: Angle in degrees

    Returns:
        Angle wrapped to [-180, 180)
    """
    return (angle_deg + 180) % 360 - 180


def angle_difference(target_deg, current_deg):
    """
    Calculate the signed shortest angular distance from current to target.

    Args:
        target_deg: Target angle in degrees
        current_deg: Current angle in degrees

    Returns:
        Signed difference in degrees [-180, 180)
        - Positive: target is clockwise from current
        - Negative: target is counterclockwise from current
    """
    return wrap_angle(target_deg - current_deg)


def clamp(value, min_val, max_val):
    """
    Clamp a value between min and max.

    Args:
        value: Value to clamp
        min_val: Minimum allowed value
        max_val: Maximum allowed value

    Returns:
        Clamped value
    """
    return max(min_val, min(value, max_val))


# =========================
# SENSOR INTERFACE
# =========================

class Sensors:
    """Interface for reading robot sensors."""

    @staticmethod
    def yaw_deg():
        """
        Get robot yaw heading in degrees [-180, +180].
        Positive rotation is clockwise when viewed from above.

        Returns:
            Current yaw angle in degrees
        """
        return motion_sensor.tilt_angles()[0] / -10.0

    @staticmethod
    def left_wheel_deg():
        """
        Get left motor position in degrees (forward is positive).

        Returns:
            Left motor position in degrees
        """
        return -motor.relative_position(RobotConfig.LEFT_MOTOR)

    @staticmethod
    def right_wheel_deg():
        """
        Get right motor position in degrees (forward is positive).

        Returns:
            Right motor position in degrees
        """
        return motor.relative_position(RobotConfig.RIGHT_MOTOR)

    @staticmethod
    def average_wheel_deg():
        """
        Get average wheel rotation in degrees (forward is positive).

        Returns:
            Average of left and right wheel positions
        """
        return (Sensors.left_wheel_deg() + Sensors.right_wheel_deg()) / 2.0

    @staticmethod
    def reset_wheels():
        """Reset both wheel encoders to zero."""
        motor.reset_relative_position(RobotConfig.LEFT_MOTOR, 0)
        motor.reset_relative_position(RobotConfig.RIGHT_MOTOR, 0)

    @staticmethod
    def reset_yaw():
        """Reset yaw sensor to zero."""
        motion_sensor.reset_yaw(0)


# =========================
# UNIT CONVERSION
# =========================

def cm_to_wheel_degrees(distance_cm):
    """
    Convert linear distance in cm to wheel rotation in degrees.

    Args:
        distance_cm: Distance in centimeters

    Returns:
        Wheel rotation in degrees
    """
    circumference = math.pi * RobotConfig.WHEEL_DIAMETER_CM
    return (distance_cm * RobotConfig.DISTANCE_GAIN / circumference) * 360.0


# =========================
# MOTION CONTROL
# =========================

class MotionController:
    """Core motion control algorithms."""

    @staticmethod
    async def _turn_with_direction(target_deg, spin_direction):
        """
        Low-level turn function that rotates in a specific direction only.
        Includes overshoot prevention and smooth deceleration.

        Args:
            target_deg: Target absolute heading
            spin_direction: +1 for clockwise, -1 for counterclockwise
        """
        target = wrap_angle(target_deg)
        stable_count = 0

        while True:
            current = Sensors.yaw_deg()
            signed_error = angle_difference(target, current)
            error_magnitude = abs(signed_error)

            # Overshoot detection: stop if we've passed the target
            if spin_direction > 0:# Clockwise
                if signed_error <= 0:# At or past target clockwise
                    motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)
                    stable_count += 1
                    if stable_count >= RobotConfig.TURN_STABLE_COUNT:
                        break
                    await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)
                    continue
            else:# Counterclockwise
                if signed_error >= 0:# At or past target counterclockwise
                    motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)
                    stable_count += 1
                    if stable_count >= RobotConfig.TURN_STABLE_COUNT:
                        break
                    await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)
                    continue

            # Tolerance check
            if error_magnitude <= RobotConfig.TURN_TOLERANCE:
                motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)
                stable_count += 1
                if stable_count >= RobotConfig.TURN_STABLE_COUNT:
                    break
                await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)
                continue
            else:
                stable_count = 0

            # Calculate turn speed with proportional control
            raw_speed = RobotConfig.TURN_KP * error_magnitude

            # Taper speed as approaching target
            if error_magnitude < RobotConfig.TURN_NEAR_THRESHOLD:
                taper_fraction = error_magnitude / RobotConfig.TURN_NEAR_THRESHOLD
                speed_cap = (RobotConfig.TURN_MIN_SPEED +
                        taper_fraction * (RobotConfig.TURN_MAX_SPEED -
                                        RobotConfig.TURN_MIN_SPEED))
            else:
                speed_cap = RobotConfig.TURN_MAX_SPEED

            # Extra slow in final approach
            if error_magnitude < RobotConfig.TURN_FINAL_THRESHOLD:
                speed_cap = min(speed_cap, 140)

            # Apply speed limits
            command_speed = clamp(raw_speed,
                                RobotConfig.TURN_MIN_SPEED,
                                min(speed_cap, RobotConfig.TURN_MAX_SPEED))
            command_speed = int(command_speed)

            # Convert spin direction to motor_pair steering value
            # +100 = clockwise, -100 = counterclockwise
            steer_value = 100 if spin_direction > 0 else -100
            steer_value *= RobotConfig.TURN_DIRECTION_SIGN

            motor_pair.move(RobotConfig.MOTOR_PAIR_ID, steer_value,
                        velocity=command_speed)

            await runloop.sleep_ms(RobotConfig.LOOP_PERIOD_MS)

        motor_pair.stop(RobotConfig.MOTOR_PAIR_ID)

    @staticmethod
    async def _drive_with_pd_steering(distance_cm, speed_limit, target_heading):
        """
        Drive a distance while holding heading using PD control.
        Uses trapezoidal velocity profile with dynamic braking distance.

        Args:
            distance_cm: Distance to travel (+ forward, - backward)
            speed_limit: Maximum wheel speed in deg/sec
            target_heading: Absolute heading to maintain
        """
        target_heading = wrap_angle(target_heading)
        target_wheel_deg = abs(cm_to_wheel_degrees(distance_cm))
        direction = 1 if distance_cm >= 0 else -1

        max_speed = min(abs(speed_limit), RobotConfig.MAX_SPEED_STRAIGHT)

        Sensors.reset_wheels()

        # Motion state
        current_velocity = 0.0
        previous_heading_error = angle_difference(target_heading, Sensors.yaw_deg())

        while True:
            progress = abs(Sensors.average_wheel_deg())
            remaining = max(target_wheel_deg - progress, 0.0)

            if remaining <= 1.0:
                break

            # PD Steering Control
            heading_error = angle_difference(target_heading, Sensors.yaw_deg())
            heading_derivative = ((heading_error - previous_heading_error) /
                                RobotConfig.LOOP_PERIOD_SEC)
            previous_heading_error = heading_error

            steering_raw = (RobotConfig.KP_HEADING * heading_error +
                        RobotConfig.KD_HEADING * heading_derivative)

            steering_command = clamp(steering_raw,
                                    -RobotConfig.STEER_LIMIT,
                                    RobotConfig.STEER_LIMIT)

            # Adapt steering for motor_pair API and direction
            steering_command = (RobotConfig.STEER_SIGN *
                            (-steering_command) * direction)
            steering_command = int(steering_command)

            # Trapezoidal Profile with Braking Distance
            braking_velocity = math.sqrt(max(0.0,
                2.0 * RobotConfig.DECEL_STRAIGHT * remaining))

            target_velocity = min(max_speed, braking_velocity)
            target_velocity = max(target_velocity, RobotConfig.MIN_SPEED_STRAIGHT)

            # Ramp velocity toward target
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


# =========================
# PUBLIC API
# =========================

def calibrate():
    """
    Calibrate the robot at the start of a run.
    Call this once at the beginning in base position.

    - Pairs the motor pair for the drive base
    - Zeros the gyro sensor (current direction becomes 0°)
    - Resets wheel encoders
    - Sets current heading to 0°
    """
    # CRITICAL: Pair the motors first!
    motor_pair.pair(RobotConfig.MOTOR_PAIR_ID,
                    RobotConfig.LEFT_MOTOR,
                    RobotConfig.RIGHT_MOTOR)

    # Reset sensors
    Sensors.reset_yaw()
    Sensors.reset_wheels()

    # Initialize state
    _state.set_heading(0.0)
    _state.is_calibrated = True


async def turn_right(target_heading_deg):
    """
    Turn CLOCKWISE to an absolute heading.

    Example:
        await turn_right(45)# Turn clockwise until facing 45°
        await turn_right(-90)# Turn clockwise until facing -90° (270°)

    Args:
        target_heading_deg: Absolute target heading in degrees
    """
    target = wrap_angle(target_heading_deg)
    await MotionController._turn_with_direction(target, spin_direction=+1)
    _state.set_heading(target)


async def turn_left(target_heading_deg):
    """
    Turn COUNTERCLOCKWISE to an absolute heading.

    Example:
        await turn_left(0)# Turn counterclockwise until facing 0°
        await turn_left(-135)# Turn counterclockwise until facing -135°

    Args:
        target_heading_deg: Absolute target heading in degrees
    """
    target = wrap_angle(target_heading_deg)
    await MotionController._turn_with_direction(target, spin_direction=-1)
    _state.set_heading(target)


async def turn_to(target_heading_deg):
    """
    Turn to an absolute heading using the SHORTEST path.
    Automatically chooses clockwise or counterclockwise.

    Example:
        await turn_to(90)# Turn shortest way to 90°
        await turn_to(180)# Turn shortest way to 180°

    Args:
        target_heading_deg: Absolute target heading in degrees
    """
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
    """
    Drive a specified distance while holding an absolute heading.

    Uses PD steering control and trapezoidal velocity profile for smooth,
    accurate motion. Positive distance drives forward, negative drives backward.

    Example:
        await drive_cm(50, speed=700, heading=0)# Drive 50cm forward at 0°
        await drive_cm(-30, speed=500, heading=45)# Drive 30cm back at 45°

    Args:
        distance_cm: Distance to travel in centimeters (+ forward, - backward)
        speed: Optional speed limit in deg/sec (default: MAX_SPEED_STRAIGHT)
        heading: Absolute heading to maintain (default: current heading)
    """
    if speed is None:
        speed = RobotConfig.MAX_SPEED_STRAIGHT

    if heading is None:
        target_heading = _state.get_heading()
    else:
        target_heading = wrap_angle(heading)

    await MotionController._drive_with_pd_steering(distance_cm, speed, target_heading)


async def drive_forward(distance_cm, speed=None, heading=None):
    """Alias for drive_cm with positive distance."""
    await drive_cm(distance_cm, speed, heading)


async def drive_backward(distance_cm, speed=None, heading=None):
    """Alias for drive_cm with negative distance."""
    await drive_cm(-abs(distance_cm), speed, heading)


# =========================
# UTILITY FUNCTIONS
# =========================

def get_current_heading():
    """
    Get the robot's current tracked heading.

    Returns:
        Current heading in degrees [-180, 180]
    """
    return _state.get_heading()


def get_sensor_heading():
    """
    Get the raw sensor yaw reading.

    Returns:
        Sensor yaw in degrees [-180, 180]
    """
    return Sensors.yaw_deg()


# =========================
# EXAMPLE USAGE
# =========================

async def example_mission():
    """
    Example mission demonstrating library usage.
    Uncomment and modify for your robot's tasks.
    """
    # Always calibrate first!
    calibrate()

    # Drive forward 60cm at 0° heading
    await drive_cm(60, speed=700, heading=0)

    # Turn clockwise to 90°
    await turn_right(90)

    # Drive 30cm at 90° heading
    await drive_cm(30, speed=600, heading=90)

    # Turn counterclockwise back to 0°
    await turn_left(0)

    # Drive backward 20cm at 0° heading
    await drive_cm(-20, speed=500, heading=0)

    # Turn to 180° using shortest path
    await turn_to(180)


async def main():
    """Main program entry point."""

    # Always calibrate first!
    calibrate()

    # ─── Mission Sequence ───

    # Initial approach with micro-adjustments
    await drive_cm(62, 500, 0)

    # Repeated precise shuttles for mechanism
    for _ in range(6):
        await drive_cm(-3, 300, 0)
        await runloop.sleep_ms(10)
        await drive_cm(3, 300, 0)
        await runloop.sleep_ms(10)

    # Accessory action
    await motor.run_for_degrees(port.E, 90, 720)

    # Return sequence
    await drive_cm(-20, 300, 0)
    motor.run_for_degrees(port.E, -80, 720)

    # Navigate to next position
    await turn_left(-45)
    await drive_cm(11.5, 300, -45)
    await turn_right(0)
    await drive_cm(27, 1000, 0)

    # Final return to base
    motor_pair.move_for_degrees(
        RobotConfig.MOTOR_PAIR_ID,
        -2300,
        15,
        velocity=1100,
        acceleration=10000
    )


# Entry point
runloop.run(main())
