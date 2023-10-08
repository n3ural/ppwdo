from abc import ABC, abstractmethod


class Robot:

    def __init__(self, base, initial_pose=(0, 0, 0)):

        self.polygon = base
        self.pose = initial_pose
        self.sensors = []

    def step_motion(self, dt):
        """
        Simulate the robot's motion over the given time interval.
        """

        # Apply dynamics to the moving parts
        self.apply_dynamics(dt)

        # Update global geometry
        self.polygon.transform(self.pose)

        # Update all the sensor
        for sensor in self.sensors:
            sensor.update_position()

    @abstractmethod
    def apply_dynamics(self, dt):
        """
        Defines how the robot should move according to the number, arrangement
        and parameters of its wheels.
        """

        pass


