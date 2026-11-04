from __future__ import annotations

from typing import Any, List, Tuple

from connections import utils
from connections.arduino import Arduino


class Serial:

    def __init__(self, robot: Any = None) -> None:
        """`robot` : instance Webots en simulation, None sur le robot réel."""
        self.simulation = robot is not None

        if self.simulation:
            self.robot = robot
            self.motors: List[Any] = []
            motors_names = ["Motor2", "Motor1", "Motor4", "Motor3"]
            for name in motors_names:
                motor = self.robot.getDevice(name)
                motor.setPosition(float("inf"))
                motor.setVelocity(0.0)
                self.motors.append(motor)
        else:
            self.arduino = Arduino(utils.SERIAL_PORT)
            self.arduino.connect()

    def disconnect(self) -> None:
        if not self.simulation:
            self.arduino.disconnect()

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        if self.simulation:
            self.motors[0].setVelocity(fl * utils.WEBOTS_SPEED_SCALE)
            self.motors[1].setVelocity(fr * utils.WEBOTS_SPEED_SCALE)
            self.motors[2].setVelocity(rl * utils.WEBOTS_SPEED_SCALE)
            self.motors[3].setVelocity(rr * utils.WEBOTS_SPEED_SCALE)
        else:
            self.arduino.SendSpeeds(*(int(speed * utils.PWM_MAX) for speed in (fl, fr, rr, rl)))

    def get_speeds_tick_per_second(self) -> Tuple[int, int, int, int]:
        if not self.simulation:
            return self.arduino.GetSpeed()
        return 0, 0, 0, 0
