from arduino import Arduino


class Serial:

    def connect(self):
        self.arduino = Arduino('/dev/ttyACM1')
        self.arduino.connect()

    def disconnect(self):
        self.arduino.disconnect()

    def send_wheel_speeds(self, fl, fr, rr, rl):
        self.arduino.SendSpeeds(fl, fr, rr, rl)

    def get_speeds_tick_per_second(self):
        return self.arduino.GetSpeed()
