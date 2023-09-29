from math import pi, atan2
from utils import linalg2_util as linalg


class AvoidObstaclesController:
    """
    The AvoidObstaclesController class offers a control strategy for a robot to detect and navigate around obstacles
    in its environment. By leveraging sensor data and a Proportional-Integral-Derivative (PID) control mechanism,
    the controller calculates and updates the robot's heading and velocities in real-time, ensuring it avoids collisions
    while progressing towards its target. The class operates in conjunction with a supervisor, which provides essential
    data like sensor placements, maximum velocities, and time stamps.
    """

    def __init__(self, supervisor):
        """
        Initialize the AvoidObstaclesController with a reference to the supervisor.

        Sets up sensor placements, gains, and control parameters essential for the obstacle avoidance mechanism.
        It also initializes key vectors and data structures for the controller's operation.

        Parameters
        ----------
        supervisor : object
            The supervisor instance that this controller will interact with and obtain necessary data from.
        """

        # bind the supervisor
        self.supervisor = supervisor

        # sensor placements
        self.proximity_sensor_placements = supervisor.proximity_sensor_placements()

        # sensor gains (weights)
        self.sensor_gains = [
            1.0 + ((0.4 * abs(p.theta)) / pi)
            for p in supervisor.proximity_sensor_placements()
        ]

        # control gains
        self.kP = 10.0
        self.kI = 0.0
        self.kD = 0.0

        # stored values - for computing next results
        self.prev_time = 0.0
        self.prev_eP = 0.0
        self.prev_eI = 0.0

        # key vectors and data (initialize to any non-zero vector)
        self.obstacle_vectors = [[1.0, 0.0]] * len(self.proximity_sensor_placements)
        self.ao_heading_vector = [1.0, 0.0]

    def update_heading(self):
        """
        Update the heading and obstacle vectors of the robot.

        Computes and stores the new heading and obstacle vectors, ensuring the robot's orientation
        is adjusted based on the obstacles detected in its proximity.
        """

        # generate and store new heading and obstacle vectors
        (
            self.ao_heading_vector,
            self.obstacle_vectors,
        ) = self.calculate_ao_heading_vector()

    def execute(self):
        """
        Implement the obstacle avoidance control strategy.

        This method computes the robot's translational and angular velocities based on detected obstacles and desired heading.
        It employs a PID control loop to adjust the robot's direction in real-time, ensuring a collision-free path.
        The velocities are then set for the robot's supervisor to act upon.
        """

        # calculate the time that has passed since the last control iteration
        current_time = self.supervisor.time()
        dt = current_time - self.prev_time

        # calculate the error terms
        theta_d = atan2(self.ao_heading_vector[1], self.ao_heading_vector[0])
        eP = theta_d
        eI = self.prev_eI + eP * dt
        eD = (eP - self.prev_eP) / dt

        # calculate angular velocity
        omega = self.kP * eP + self.kI * eI + self.kD * eD

        # calculate translational velocity
        # velocity is v_max when omega is 0,
        # drops rapidly to zero as |omega| rises
        v = self.supervisor.v_max() / (abs(omega) + 1) ** 2

        # store values for next control iteration
        self.prev_time = current_time
        self.prev_eP = eP
        self.prev_eI = eI

        self.supervisor.set_outputs(v, omega)

        # === FOR DEBUGGING ===
        # self._print_vars( eP, eI, eD, v, omega )

    # return a obstacle avoidance vector in the robot's reference frame
    # also returns vectors to detected obstacles in the robot's reference frame

    def calculate_ao_heading_vector(self):
        """
        Compute the Avoid-Obstacles (AO) heading vector for the robot.

        This method calculates the heading vector that aids the robot in avoiding obstacles detected in its vicinity.
        It factors in sensor data, distances to obstacles, and sensor placements to derive a directional vector that
        steers the robot away from potential collisions. The computed AO heading vector is then normalized and stored
        for subsequent control operations.
        """

        # initialize vector
        obstacle_vectors = [[0.0, 0.0]] * len(self.proximity_sensor_placements)
        ao_heading_vector = [0.0, 0.0]

        # get the distances indicated by the robot's sensor readings
        sensor_distances = self.supervisor.proximity_sensor_distances()

        # calculate the position of detected obstacles and find an avoidance vector
        robot_pos, robot_theta = self.supervisor.estimated_pose().vunpack()
        for i in range(len(sensor_distances)):
            # calculate the position of the obstacle
            sensor_pos, sensor_theta = self.proximity_sensor_placements[i].vunpack()
            vector = [sensor_distances[i], 0.0]
            vector = linalg.rotate_and_translate_vector(
                vector, sensor_theta, sensor_pos
            )
            obstacle_vectors[
                i
            ] = vector  # store the obstacle vectors in the robot's reference frame

            # accumluate the heading vector within the robot's reference frame
            ao_heading_vector = linalg.add(
                ao_heading_vector, linalg.scale(vector, self.sensor_gains[i])
            )

        return ao_heading_vector, obstacle_vectors

    def _print_vars(self, eP, eI, eD, v, omega):
        print("\n\n")
        print("==============")
        print("ERRORS:")
        print("eP: " + str(eP))
        print("eI: " + str(eI))
        print("eD: " + str(eD) + "\n")
        print("CONTROL COMPONENTS:")
        print("kP * eP = " + str(self.kP) + " * " + str(eP))
        print("= " + str(self.kP * eP))
        print("kI * eI = " + str(self.kI) + " * " + str(eI))
        print("= " + str(self.kI * eI))
        print("kD * eD = " + str(self.kD) + " * " + str(eD))
        print("= " + str(self.kD * eD) + "\n")
        print("OUTPUTS:")
        print("omega: " + str(omega))
        print("v    : " + str(v))
