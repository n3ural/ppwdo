from math import atan2
from baseline.utils import linalg2_util as linalg


class GoToGoalController:
    """
    The GoToGoalController class facilitates a control strategy for directing a robot towards a specific target or goal position.
    Through a Proportional-Integral-Derivative (PID) control mechanism, it computes the necessary angular and translational velocities
    to guide the robot towards its destination. The controller collaborates with a supervisor that provides essential data such as
    the robot's current pose, maximum velocities, and timestamps.
    """

    def __init__(self, supervisor):
        """
        Initialize the GoToGoalController with a reference to the supervisor.

        Sets up control gains (proportional, integral, and derivative), initializes storage values for control computations,
        and readies a key vector for heading towards the goal.

        Parameters
        ----------
        supervisor : object
            The supervisor instance that this controller will interact with and obtain necessary data from.
        """

        # bind the supervisor
        self.supervisor = supervisor

        # gains
        self.kP = 5.0
        self.kI = 0.0
        self.kD = 0.0

        # stored values - for computing next results
        self.prev_time = 0.0
        self.prev_eP = 0.0
        self.prev_eI = 0.0

        # key vectors and data (initialize to any non-zero vector)
        self.gtg_heading_vector = [1.0, 0.0]

    def update_heading(self):
        """
        Generate and store the robot's new heading vector.

        Utilizes the calculate_gtg_heading_vector method to compute a heading vector guiding the robot towards its goal.
        This vector is then stored for subsequent control operations.
        """

        # generate and store new heading vector
        self.gtg_heading_vector = self.calculate_gtg_heading_vector()

    def execute(self):
        """
        Implement the control strategy to guide the robot towards its goal.

        Computes the time elapsed since the last control iteration, derives error terms for PID control,
        calculates the angular and translational velocities, and sets the robot's outputs to move in the
        direction of the target goal.
        """

        # calculate the time that has passed since the last control iteration
        current_time = self.supervisor.time()
        dt = current_time - self.prev_time

        # calculate the error terms
        theta_d = atan2(self.gtg_heading_vector[1], self.gtg_heading_vector[0])
        eP = theta_d
        eI = self.prev_eI + eP * dt
        eD = (eP - self.prev_eP) / dt

        # calculate angular velocity
        omega = self.kP * eP + self.kI * eI + self.kD * eD

        # calculate translational velocity
        # velocity is v_max when omega is 0,
        # drops rapidly to zero as |omega| rises
        v = self.supervisor.v_max() / (abs(omega) + 1) ** 0.5

        # store values for next control iteration
        self.prev_time = current_time
        self.prev_eP = eP
        self.prev_eI = eI

        self.supervisor.set_outputs(v, omega)

        # === FOR DEBUGGING ===
        # self._print_vars( eP, eI, eD, v, omega )

    # return a go-to-goal heading vector in the robot's reference frame

    def calculate_gtg_heading_vector(self):
        """
        Compute a heading vector directing the robot towards its goal.

        Based on the robot's current pose and the target position, this method determines a directional vector in the
        robot's reference frame that guides it towards the goal. The vector aids in adjusting the robot's orientation
        and movement to approach its destination.

        Returns
        -------
        list
            A heading vector in the robot's reference frame, pointing towards the goal.
        """

        # get the inverse of the robot's pose
        robot_inv_pos, robot_inv_theta = (
            self.supervisor.estimated_pose().inverse().vunpack()
        )

        # calculate the goal vector in the robot's reference frame
        goal = self.supervisor.goal()
        goal = linalg.rotate_and_translate_vector(goal, robot_inv_theta, robot_inv_pos)

        return goal

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
