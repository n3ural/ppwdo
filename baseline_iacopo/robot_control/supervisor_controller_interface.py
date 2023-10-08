# an interfacing allowing a controller to interact with its supervisor
class SupervisorControllerInterface:
    """
    The SupervisorControllerInterface serves as an intermediary between a robot's controller and its supervisor.
    Through this interface, the controller can access various attributes, states, and functionalities
    of the supervisor without directly interacting with the supervisor's internals.
    The interface provides methods to fetch current states, sensor readings, and other vital details
    from the supervisor, enabling the controller to make informed decisions.
    """

    def __init__(self, supervisor):
        """
        Initialize the SupervisorControllerInterface with a reference to the supervisor.

        Parameters
        ----------
        supervisor : object
            The supervisor instance that this interface will interact with.
        """

        self.supervisor = supervisor

    # get the current control state
    def current_state(self):
        """
        Fetch the current operational state of the supervisor's state machine.

        The supervisor may operate in various states like 'go-to-goal', 'avoid-obstacles', etc.
        This method provides the active state, allowing the controller to understand the supervisor's current behavior.

        Returns
        -------
        str
            The active state of the supervisor's state machine.
        """

        return self.supervisor.state_machine.current_state

    # get the supervisor's internal pose estimation
    def estimated_pose(self):
        """
        Retrieve the supervisor's internal estimation of the robot's pose.

        The supervisor keeps an ongoing estimate of the robot's position and orientation based on sensor data.
        This method provides that estimate to the controller.

        Returns
        -------
        tuple
            The robot's estimated pose (position and orientation).
        """

        return self.supervisor.estimated_pose

    # get the placement poses of the robot's sensors
    def proximity_sensor_placements(self):
        """
        Get the positions where the robot's proximity sensors are placed.

        Returns
        -------
        list
            A list of positions (typically in the form of (x, y) coordinates) representing
            the placements of the robot's proximity sensors.
        """

        return self.supervisor.proximity_sensor_placements

    # get the robot's proximity sensor read values converted to real distances in meters
    def proximity_sensor_distances(self):
        """
        Obtain the distances detected by the robot's proximity sensors.

        Proximity sensors detect obstacles and their distances from the robot.
        This method provides these distances to the controller, enabling it to make decisions
        based on the robot's surroundings.

        Returns
        -------
        list
            A list of distances (in meters) detected by each proximity sensor.
        """

        return self.supervisor.proximity_sensor_distances

    # get true/false indicators for which sensors are actually detecting obstacles
    def proximity_sensor_positive_detections(self):
        """
        Determine which proximity sensors are actively detecting obstacles.

        By comparing sensor readings against a maximum range, this method identifies which sensors
        have detected obstacles within their effective range.

        Returns
        -------
        list
            A list of boolean values, with each entry indicating whether the corresponding sensor
            has detected an obstacle (True) or not (False).
        """

        sensor_range = self.supervisor.proximity_sensor_max_range
        return [d < sensor_range - 0.001 for d in self.proximity_sensor_distances()]

    # get the velocity limit of the supervisor
    def v_max(self):
        """
        Fetch the supervisor's maximum allowable velocity for the robot.

        Returns
        -------
        float
            The maximum velocity (in m/s) that the supervisor permits for the robot.
        """

        return self.supervisor.v_max

    # get the supervisor's goal
    def goal(self):
        """
        Obtain the supervisor's target goal position for the robot.

        Returns
        -------
        tuple
            The target goal position (typically in the form of (x, y) coordinates).
        """

        return self.supervisor.goal

    # get the supervisor's internal clock time
    def time(self):
        """
        Retrieve the supervisor's internal clock time.

        This provides the controller with an understanding of the elapsed time since the supervisor started.

        Returns
        -------
        float
            The current time on the supervisor's internal clock (in seconds).
        """

        return self.supervisor.time

    # set the outputs of the supervisor
    def set_outputs(self, v, omega):
        """
        Assign the supervisor's output velocities.

        This method allows the controller to set the linear and angular velocities for the robot,
        which the supervisor will then attempt to achieve.

        Parameters
        ----------
        v : float
            The desired linear velocity for the robot (in m/s).
        omega : float
            The desired angular velocity for the robot (in rad/s).
        """

        self.supervisor.v_output = v
        self.supervisor.omega_output = omega
