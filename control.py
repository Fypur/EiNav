import time
from serialpi import Serial


class Control:

    def __init__(self, serial: Serial) -> None:
        self.serial = serial
        self.kp = 60

        self.turn_duration = 0.5
        self.turn_speed = 0.8

    def stop(self):
        """Stops all motors."""
        self.serial.send_wheel_speeds(0.0, 0.0, 0.0, 0.0)

    def forward(self, speed: float = 0.8):
        """Moves robot forward."""
        self.serial.send_wheel_speeds(speed, speed, speed, speed)

    def aim(self, error: float):
        correction = error * self.kp
        base_speed = 0.5 if not self.serial.simulation else 8.0
        left_speed = base_speed + correction
        right_speed = base_speed - correction

        self.serial.send_wheel_speeds(left_speed, right_speed, right_speed, left_speed)

    def turn_90_degrees(self, left: bool, action_start_time: float) -> bool:
        """Returns True if the duration of the turn has elapsed."""
        if left:
            left_speed = -self.turn_speed
            right_speed = self.turn_speed
        else:
            left_speed = self.turn_speed
            right_speed = -self.turn_speed

        self.serial.send_wheel_speeds(left_speed, right_speed, right_speed, left_speed)

        if self.serial.simulation:
            current_time = self.serial.robot.getTime()
        else:
            current_time = time.time()

        return current_time - action_start_time > self.turn_duration
