from baseline.utils import math_util


class GoToAngleController:
    """
    The GoToAngleController class provides a control mechanism for guiding a robot to a specific target angle or orientation.
    By employing a proportional control strategy, the class computes the necessary angular velocity for the robot
    to adjust its orientation and align with the desired angle. The controller works in conjunction with a supervisor
    that provides essential data like the robot's current estimated pose.
    """

    def __init__(self, supervisor):
        """
        Initialize the GoToAngleController with a reference to the supervisor.

        Sets up the proportional control gain and binds the provided supervisor for subsequent interactions.

        Parameters
        ----------
        supervisor : object
            The supervisor instance that this controller will interact with and obtain necessary data from.
        """

        # bind the supervisor
        self.supervisor = supervisor

        # gains
        self.k_p = 5.0

    def execute(self, theta_d):
        """
        Calculate and set the robot's angular velocity to achieve the desired orientation.

        This method computes the error between the robot's current orientation and the target angle.
        Using a proportional control strategy, it determines the required angular velocity and sets
        the robot's outputs to move forward at a constant translational velocity while adjusting its
        orientation based on the calculated angular velocity.

        Parameters
        ----------
        theta_d : float
            The desired target angle or orientation for the robot (in radians).
        """

        theta = self.supervisor.estimated_pose().theta
        e = math_util.normalize_angle(theta_d - theta)
        omega = self.k_p * e

        self.supervisor.set_outputs(1.0, omega)
