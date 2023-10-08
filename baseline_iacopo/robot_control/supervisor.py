from math import pi, log, sin, cos, radians
from baseline.models.pose import Pose
from baseline.robot_control.supervisor_controller_interface import SupervisorControllerInterface
from baseline.robot_control.supervisor_state_machine import SupervisorStateMachine
from baseline.robot_control.controllers.avoid_obstacles_controller import AvoidObstaclesController
from baseline.robot_control.controllers.follow_wall_controller import FollowWallController
from baseline.robot_control.controllers.go_to_angle_controller import GoToAngleController
from baseline.robot_control.controllers.go_to_goal_controller import GoToGoalController
from baseline.robot_control.controllers.gtg_and_ao_controller import GTGAndAOController

# control parameters
K3_TRANS_VEL_LIMIT = 0.3148  # m/s
K3_ANG_VEL_LIMIT = 2.2763  # rad/s


class Supervisor:
    """
        The Supervisor class is responsible for managing and controlling a robot through a specified interface. It utilizes various controllers, updates the robot's state based on sensor readings, and sends commands to the robot.

        Attributes:
            time (float): Internal clock time in seconds.
            robot (RobotInterface): The interface through which the supervisor interacts with the robot.
            proximity_sensor_placements (list): Placement pose of the sensors on the robot body.
            proximity_sensor_max_range (float): Max detection range of the sensors.
            robot_wheel_radius (float): The radius of a drive wheel on the robot.
            robot_wheel_base_length (float): The robot's wheel base.
            wheel_encoder_ticks_per_revolution (int): The number of wheel encoder ticks per revolution of a drive wheel.
            prev_ticks_left (int): Previous tick count for the left wheel.
            prev_ticks_right (int): Previous tick count for the right wheel.
            go_to_angle_controller (Controller): Controller for going to a specified angle.
            go_to_goal_controller (Controller): Controller for going to a specified goal.
            avoid_obstacles_controller (Controller): Controller for avoiding obstacles.
            gtg_and_ao_controller (Controller): Controller combining go-to-goal and avoid-obstacles behaviors.
            follow_wall_controller (Controller): Controller for following walls.
            state_machine (SupervisorStateMachine): State machine managing the states of the supervisor.
            proximity_sensor_distances (list): Sensor distances.
            estimated_pose (Pose): Estimated pose of the robot.
            current_controller (Controller): The currently active controller.
            goal (list): The goal to which the supervisor will guide the robot.
            v_max (float): Maximum translational velocity.
            omega_max (float): Maximum angular velocity.
            v_output (float): Output translational velocity.
            omega_output (float): Output angular velocity.

        Methods:
            step(dt: float): Simulate the supervisor running for one time increment.
            execute(): Execute one control loop, updating the state and applying the current controller.
            _update_state(): Update the estimated robot state and the control state.
            _update_controller_headings(): Calculate updated heading vectors for the active controllers.
            _update_proximity_sensor_distances(): Update the distances indicated by the proximity sensors.
            _update_odometry(): Update the estimated position of the robot using its wheel encoder readings.
            _send_robot_commands(): Generate and send the correct commands to the robot.
            _uni_to_diff(v: float, omega: float) -> Tuple[float, float]: Convert unicycle model parameters to differential model parameters.
            _diff_to_uni(v_l: float, v_r: float) -> Tuple[float, float]: Convert differential model parameters to unicycle model parameters.
        """
    def __init__(
        self,
        robot_interface,  # the interface through which this supervisor will interact with the robot
        wheel_radius,  # the radius of a drive wheel on the robot
        wheel_base_length,  # the robot's wheel base
        wheel_encoder_ticks_per_rev,  # the number of wheel encoder ticks per revolution of a drive wheel
        sensor_placements,  # placement pose of the sensors on the robot body
        sensor_range,  # max detection range of the sensors
        goal=[0.0, 0.0],  # the goal to which this supervisor will guide the robot
        initial_pose_args=[0.0, 0.0, 0.0],
    ):  # the pose the robot will have when control begins

        # internal clock time in seconds
        self.time = 0.0

        # robot representation
        # NOTE: the supervisor does NOT have access to the physical robot, only the
        # robot's interface
        self.robot = robot_interface

        # proximity sensor information
        self.proximity_sensor_placements = [
            Pose(rawpose[0], rawpose[1], radians(rawpose[2]))
            for rawpose in sensor_placements
        ]
        self.proximity_sensor_max_range = sensor_range

        # odometry information
        self.robot_wheel_radius = wheel_radius
        self.robot_wheel_base_length = wheel_base_length
        self.wheel_encoder_ticks_per_revolution = wheel_encoder_ticks_per_rev
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

        # controllers
        controller_interface = SupervisorControllerInterface(self)
        self.go_to_angle_controller = GoToAngleController(controller_interface)
        self.go_to_goal_controller = GoToGoalController(controller_interface)
        self.avoid_obstacles_controller = AvoidObstaclesController(controller_interface)
        self.gtg_and_ao_controller = GTGAndAOController(controller_interface)
        self.follow_wall_controller = FollowWallController(controller_interface)

        # state machine
        self.state_machine = SupervisorStateMachine(self)

        # state
        self.proximity_sensor_distances = [0.0, 0.0] * len(
            sensor_placements
        )  # sensor distances
        self.estimated_pose = Pose(*initial_pose_args)  # estimated pose
        self.current_controller = self.go_to_goal_controller  # current controller

        # goal
        self.goal = goal

        # control bounds
        self.v_max = K3_TRANS_VEL_LIMIT
        self.omega_max = K3_ANG_VEL_LIMIT

        # CONTROL OUTPUTS - UNICYCLE
        self.v_output = 0.0
        self.omega_output = 0.0

    # simulate this supervisor running for one time increment

    def step(self, dt):
        """
        Simulate the supervisor's behavior over a specified time interval.

        This function increments the internal clock by the given time duration 
        and then invokes the main execution loop to simulate the supervisor's activities 
        over that time span.

        Parameters
        ----------
        dt : float
            The duration to advance the simulation, in seconds.
        """
        # increment the internal clock time
        self.time += dt

        # NOTE: For simplicity, we assume that the onboard computer executes exactly
        # one control loop for every simulation time increment. Although technically
        # this is not likely to be realistic, it is a good simplificiation

        # execute one full control loop
        self.execute()

    # execute one control loop
    def execute(self):
        """
        Execute one control loop cycle for the supervisor.

        During each cycle, the supervisor's state is updated based on the 
        latest sensor readings. The currently active controller logic is applied,
        and the resultant commands are sent to the robot.
        """
        self._update_state()  # update state
        self.current_controller.execute()  # apply the current controller
        self._send_robot_commands()  # output the generated control signals to the robot

    # update the estimated robot state and the control state
    def _update_state(self):
        """
        Update the internal state of the supervisor.

        The function retrieves the latest sensor data, computes the new estimated 
        robot state, recalculates the heading for the active controllers, and then updates 
        the current control state using the state machine.
        """
        # update estimated robot state from sensor readings
        self._update_proximity_sensor_distances()
        self._update_odometry()

        # calculate new heading vectors for each controller
        self._update_controller_headings()

        # update the control state
        self.state_machine.update_state()

    # calculate updated heading vectors for the active controllers
    def _update_controller_headings(self):
        """
        Refresh the heading directives for all active controllers.

        Each controller has its own strategy for determining its heading 
        based on the robot's current state. This function triggers each controller 
        to recompute its heading.
        """
        self.go_to_goal_controller.update_heading()
        self.avoid_obstacles_controller.update_heading()
        self.gtg_and_ao_controller.update_heading()
        self.follow_wall_controller.update_heading()

    # update the distances indicated by the proximity sensors
    def _update_proximity_sensor_distances(self):
        """
        Retrieve and process proximity sensor readings.

        The function fetches the latest readings from the robot's proximity sensors 
        and processes them to determine the distances to nearby objects.
        """
        self.proximity_sensor_distances = [
            0.02 - (log(readval / 3960.0)) / 30.0
            for readval in self.robot.read_proximity_sensors()
        ]

    # update the estimated position of the robot using it's wheel encoder readings
    def _update_odometry(self):
        """
        Compute the robot's estimated pose using wheel encoder data.

        The function reads the wheel encoder values, determines the change in encoder ticks 
        since the last update, and then calculates the robot's estimated movement. Using this 
        movement data, the robot's new estimated pose is computed.
        """
        R = self.robot_wheel_radius
        N = float(self.wheel_encoder_ticks_per_revolution)

        # read the wheel encoder values
        ticks_left, ticks_right = self.robot.read_wheel_encoders()

        # get the difference in ticks since the last iteration
        d_ticks_left = ticks_left - self.prev_ticks_left
        d_ticks_right = ticks_right - self.prev_ticks_right

        # estimate the wheel movements
        d_left_wheel = 2 * pi * R * (d_ticks_left / N)
        d_right_wheel = 2 * pi * R * (d_ticks_right / N)
        d_center = 0.5 * (d_left_wheel + d_right_wheel)

        # calculate the new pose
        prev_x, prev_y, prev_theta = self.estimated_pose.sunpack()
        new_x = prev_x + (d_center * cos(prev_theta))
        new_y = prev_y + (d_center * sin(prev_theta))
        new_theta = prev_theta + (
            (d_right_wheel - d_left_wheel) / self.robot_wheel_base_length
        )

        # update the pose estimate with the new values
        self.estimated_pose.supdate(new_x, new_y, new_theta)

        # save the current tick count for the next iteration
        self.prev_ticks_left = ticks_left
        self.prev_ticks_right = ticks_right

    # generate and send the correct commands to the robot
    def _send_robot_commands(self):
        """
        Generate and transmit drive commands to the robot.

        Based on the outputs from the active controller, this function formulates the 
        drive commands while ensuring they adhere to the robot's speed constraints. 
        The commands are then dispatched to the robot for execution.
        """
        # limit the speeds:
        v = max(min(self.v_output, self.v_max), -self.v_max)
        omega = max(min(self.omega_output, self.omega_max), -self.omega_max)

        # send the drive commands to the robot
        v_l, v_r = self._uni_to_diff(v, omega)
        self.robot.set_wheel_drive_rates(v_l, v_r)

    def _uni_to_diff(self, v, omega):
        """
        Convert unicycle motion parameters to differential drive parameters.

        Given the translational and angular velocities from a unicycle motion model, 
        this function computes the corresponding angular velocities for a differential 
        drive robot's left and right wheels.

        Parameters
        ----------
        v : float
            Translational velocity of the robot, in m/s.
        omega : float
            Angular velocity of the robot, in rad/s.

        Returns
        -------
        tuple
            Angular velocities for the left and right wheels, respectively.
        """
        # v = translational velocity (m/s)
        # omega = angular velocity (rad/s)

        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        v_l = ((2.0 * v) - (omega * L)) / (2.0 * R)
        v_r = ((2.0 * v) + (omega * L)) / (2.0 * R)

        return v_l, v_r

    def _diff_to_uni(self, v_l, v_r):
        """
        Convert differential drive parameters to unicycle motion parameters.

        Given the angular velocities for a differential drive robot's left and right wheels, 
        this function computes the corresponding translational and angular velocities 
        for a unicycle motion model.

        Parameters
        ----------
        v_l : float
            Angular velocity of the left wheel, in rad/s.
        v_r : float
            Angular velocity of the right wheel, in rad/s.

        Returns
        -------
        tuple
            Translational velocity and angular velocity for the unicycle model.
        """
        # v_l = left-wheel angular velocity (rad/s)
        # v_r = right-wheel angular velocity (rad/s)

        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        v = (R / 2.0) * (v_r + v_l)
        omega = (R / L) * (v_r - v_l)

        return v, omega
