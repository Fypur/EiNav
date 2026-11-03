from serialpi import Serial


class Control:

    def __init__(self, simulation) -> None:
        self.simulation = simulation
        self.serial = Serial()

    def apply_pid(self, target):
        pass

    def turn_90_degrees(self, left: bool):
        pass

    def send_wheel_speeds(self, fl, fr, rr, rl):
        if self.simulation:
            pass  #send webot command
        else:
            self.serial.send_wheel_speeds(fl, fr, rr, rl)
