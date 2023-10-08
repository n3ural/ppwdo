import math

from model.robot.robot import Robot
from geometry.polygon import Polygon
from geometry.point import Point
from model.robot.hardware.encoder import WheelEncoder
from model.robot.hardware.proximity import ProximitySensor


# Khepera III properties
K3_WHEEL_RADIUS = 0.021  # meters
K3_WHEEL_BASE_LENGTH = 0.0885  # meters
K3_WHEEL_TICKS_PER_REV = 2765
K3_MAX_WHEEL_DRIVE_RATE = 15.0  # rad/s

# Khepera III dimensions
K3_POINTS = [
    Point(-0.024, 0.064),
    Point(0.033, 0.064),
    Point(0.057, 0.043),
    Point(0.074, 0.010),
    Point(0.074, -0.010),
    Point(0.057, -0.043),
    Point(0.033, -0.064),
    Point(-0.025, -0.064),
    Point(-0.042, -0.043),
    Point(-0.048, -0.010),
    Point(-0.048, 0.010),
    Point(-0.042, 0.043),
]

K3_SENSOR_MIN_RANGE = 0.02
K3_SENSOR_MAX_RANGE = 0.2
K3_SENSOR_POSES = [
    (-0.038, 0.048, 128),  # x, y, theta_degrees
    (0.019, 0.064, 75),
    (0.050, 0.050, 42),
    (0.070, 0.017, 13),
    (0.070, -0.017, -13),
    (0.050, -0.050, -42),
    (0.019, -0.064, -75),
    (-0.038, -0.048, -128),
    (-0.048, 0.000, 180),
]


class Khepera(Robot):

    def __init__(self):

        super().__init__(Polygon(K3_POINTS))

        # Wheel arrangement: using two separate variables we can build a robot
        # with different sized wheels
        self.left_wheel_radius = K3_WHEEL_RADIUS
        self.right_wheel_radius = K3_WHEEL_RADIUS

        self.wheel_base_length = K3_WHEEL_BASE_LENGTH

        # Wheel rates (rad/s)
        self.left_wheel_drive_rate = 0.0
        self.right_wheel_drive_rate = 0.0

        # Add wheel encoders
        self.left_wheel_encoder = WheelEncoder(K3_WHEEL_TICKS_PER_REV)
        self.right_wheel_encoder = WheelEncoder(K3_WHEEL_TICKS_PER_REV)

        # Add distance sensors (ToF, IR, ..)
        for pose in K3_SENSOR_POSES:

            # TODO fix pose of the sensor (I don't like using it in the constructor)
            self.sensors.append(ProximitySensor(K3_SENSOR_MIN_RANGE, K3_SENSOR_MAX_RANGE, math.radians(20)))

        # Supervisor
        self.supervisor = Supervisor(
            RobotSupervisorInterface(self),
            K3_WHEEL_RADIUS,
            K3_WHEEL_BASE_LENGTH,
            K3_WHEEL_TICKS_PER_REV,
            K3_SENSOR_POSES,
            K3_SENSOR_MAX_RANGE,
        )

    def apply_dynamics(self, dt):

        # Compute the change in wheel angle (in radians)
        d_angle_left = dt * self.left_wheel_drive_rate
        d_angle_right = dt * self.right_wheel_drive_rate

        # calculate the distance traveled
        left_meters_per_rad = self.left_wheel_radius
        right_meters_per_rad = self.right_wheel_radius
        d_left_wheel = d_angle_left * left_meters_per_rad
        d_right_wheel = d_angle_right * right_meters_per_rad
        d_center = (d_left_wheel + d_right_wheel) / 2.0

        # calculate the new pose
        old_x, old_y, old_theta = self.pose
        new_x = old_x + (d_center * math.cos(old_theta))
        new_y = old_y + (d_center * math.sin(old_theta))
        new_theta = old_theta + (
            (d_right_wheel - d_left_wheel) / self.wheel_base_length
        )

        # calculate the number of rotations each wheel has made
        revolutions_left = d_angle_left / (2 * math.pi)
        revolutions_right = d_angle_right / (2 * math.pi)

        # update the state of the moving parts
        self.pose = (new_x, new_y, new_theta)
        self.left_wheel_encoder.step_revolutions(revolutions_left)
        self.right_wheel_encoder.step_revolutions(revolutions_right)
