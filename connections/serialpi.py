from __future__ import annotations

from typing import Any, List, Tuple

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
            self.arduino = Arduino('/dev/ttyACM0')
            self.arduino.connect()

    def disconnect(self) -> None:
        if not self.simulation:
            self.arduino.disconnect()

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float) -> None:
        if self.simulation:
            self.motors[0].setVelocity(fl*10)
            self.motors[1].setVelocity(fr*10)
            self.motors[2].setVelocity(rl*10)
            self.motors[3].setVelocity(rr*10)
        else:
            self.arduino.SendSpeeds((int)(fl * 255), (int)(fr * 255), (int)(rr * 255), (int)(rl * 255))

    def get_speeds_tick_per_second(self) -> Tuple[int, int, int, int]:
        if not self.simulation:
            return self.arduino.GetSpeed()
        return 0, 0, 0, 0
