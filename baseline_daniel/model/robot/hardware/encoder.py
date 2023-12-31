from robot.hardware.sensor import Sensor


class WheelEncoder(Sensor):

    def __init__(self, ticks_per_rev):

        self.ticks_per_rev = ticks_per_rev
        self.real_revs = 0.0
        self.tick_count = 0

    def step_revolutions(self, revolutions):
        """
        Update the tick count for this wheel encoder.
        Takes a float representing the number of forward revolutions made
        """

        self.real_revs += revolutions
        self.tick_count = int(self.real_revs * self.ticks_per_rev)

    def read(self):
        return self.tick_count
