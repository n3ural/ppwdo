# a class representing the available interactions a supervisor may have with a robot
class RobotSupervisorInterface:
    """
    The RobotSupervisorInterface class serves as a communication layer between a supervisor and a robot,
    allowing the supervisor to interact with and control various aspects of the robot. Through this interface,
    the supervisor can read sensor data, retrieve wheel encoder readings, and set wheel drive rates for the robot,
    thereby managing the robot's movements and responses to its environment.
    """

    def __init__(self, robot):
        """
        Initialize the RobotSupervisorInterface with a reference to the robot.

        Parameters
        ----------
        robot : object
            The robot instance that this interface will interact with and control.
        """

        self.robot = robot

    # read the proximity sensors
    def read_proximity_sensors(self):
        """
        Retrieve the readings from the robot's proximity sensors.

        The robot's proximity sensors detect nearby obstacles and their distances from the robot.
        This method returns a list of readings representing the detected distances from each sensor.

        Returns
        -------
        list
            A list of readings from the robot's proximity sensors, representing detected distances to obstacles.
        """

        return [s.read() for s in self.robot.ir_sensors]

    # read the wheel encoders
    def read_wheel_encoders(self):
        """
        Obtain the readings from the robot's wheel encoders.

        Wheel encoders measure the rotation of the robot's wheels, providing information about the robot's movement
        and traveled distance. This method returns a list of readings from each wheel encoder.

        Returns
        -------
        list
            A list of readings from the robot's wheel encoders, representing the rotation of the wheels.
        """

        return [e.read() for e in self.robot.wheel_encoders]

    # apply wheel drive command
    def set_wheel_drive_rates(self, v_l, v_r):
        """
        Set the drive rates for the robot's wheels.

        This method allows the supervisor to control the robot's movement by specifying the drive rates
        for the left and right wheels.

        Parameters
        ----------
        v_l : float
            The desired drive rate for the robot's left wheel (in m/s or rad/s).
        v_r : float
            The desired drive rate for the robot's right wheel (in m/s or rad/s).
        """

        self.robot.set_wheel_drive_rates(v_l, v_r)
