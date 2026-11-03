from serialpi import Serial
import time


class Control:

    def __init__(self, serial: Serial) -> None:
        self.serial = serial
        self.kp = 60

        self.turn_duration = 0.5
        self.turn_speed = 0.8

    def aim(self, error):
        correction = error * self.kp
        base_speed = 120
        left_speed = base_speed + correction
        right_speed = base_speed - correction

        self.serial.send_wheel_speeds(left_speed, right_speed, right_speed, left_speed)

    def turn_90_degrees(self, left: bool, action_start_time):
        """returns true if done"""

        if left:
            left_speed = -self.turn_speed
            right_speed = self.turn_speed
        else:
            left_speed = self.turn_speed
            right_speed = -self.turn_speed

        self.serial.send_wheel_speeds(left_speed, right_speed, right_speed, left_speed)

        # returns True if duration of spins is greater than turn_duration
        return (self.serial.simulation and self.serial.robot.getTime() - action_start_time > self.turn_duration) \
            or (not(self.serial.simulation) and time.time() - action_start_time > self.turn_duration)
