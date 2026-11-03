from arduino import Arduino


class Serial:

    def __init__(self, robot=None) -> None:
        self.simulation = robot is None

        if robot is None:
            self.arduino = Arduino('/dev/ttyACM1')
            self.arduino.connect()
        else:
            self.robot = robot
            self.motors = []
            motors_names = ["Motor2", "Motor1", "Motor4", "Motor3"]
            for name in motors_names:
                motor = self.robot.getDevice(name)
                motor.setPosition(float("inf"))
                motor.setVelocity(0.0)
                self.motors.append(motor)

    def disconnect(self):
        self.arduino.disconnect()

    def send_wheel_speeds(self, fl: float, fr: float, rr: float, rl: float):
        if not self.simulation:
            self.arduino.SendSpeeds((int)(fl * 255), (int)(fr * 255), (int)(rr * 255), (int)(rl * 255))
        else:
            self.motors[0].setVelocity(fl)
            self.motors[1].setVelocity(fr)
            self.motors[2].setVelocity(rl)
            self.motors[3].setVelocity(rr)

    def get_speeds_tick_per_second(self):
        return self.arduino.GetSpeed()
