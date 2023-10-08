import math

from model.robot.robot import Robot
from geometry.polygon import Polygon
from geometry.point import Point
from model.robot.hardware.encoder import WheelEncoder

# Cobalt properties
COBALT_WHEEL_RADIUS = 0.045
COBALT_WHEEL_BASE_LENGTH = 0.07
COBALT_WHEEL_TICKS_PER_REV = 2765  # TODO set this variable
COBALT_WHEEL_MAX_RATE = 15.0  # TODO set this variable

# Cobalt
COBALT_BASE_RADIUS = 0.025  # 0.25 meters = 25 cm
COBALT_POINTS = [
    Point(COBALT_BASE_RADIUS * math.sin(math.radians(angle)),
          COBALT_BASE_RADIUS * math.sin(math.radians(angle))) for angle in range(0, 360, 30)
]  # 12 points to build the model polygon

COBALT_SENSOR_MIN_RANGE = 0.02
COBALT_SENSOR_MAX_RANGE = 0.2
COBALT_SENSOR_POSES = [
    (0.07,  0.07,  45),  # x, y, theta_degrees
    (0.05,   0.0,   0),
    (0.07, -0.07, -45)
]


class Cobalt(Robot):

    def __init__(self):

        super().__init__(Polygon(COBALT_POINTS))

        # Wheel arrangement: using two separate variables we can build a robot
        # with different sized wheels
        self.left_wheel_radius = COBALT_WHEEL_RADIUS
        self.right_wheel_radius = COBALT_WHEEL_RADIUS

        self.wheel_base_length = COBALT_WHEEL_BASE_LENGTH

        # Wheel rates (rad/s)
        self.left_wheel_drive_rate = 0.0
        self.right_wheel_drive_rate = 0.0

        # Add wheel encoders
        self.left_wheel_encoder = WheelEncoder(COBALT_WHEEL_TICKS_PER_REV)
        self.right_wheel_encoder = WheelEncoder(COBALT_WHEEL_TICKS_PER_REV)

        # Add distance sensors (ToF, IR, ..)
        for pose in COBALT_SENSOR_POSES:

            # TODO fix pose of the sensor (I don't like using it in the constructor)
            self.sensors.append(ProximitySensor(COBALT_SENSOR_MIN_RANGE, COBALT_SENSOR_MAX_RANGE, math.radians(20)))

        # Supervisor
        self.supervisor = Supervisor(
            RobotSupervisorInterface(self),
            COBALT_WHEEL_RADIUS,
            COBALT_WHEEL_BASE_LENGTH,
            COBALT_WHEEL_TICKS_PER_REV,
            COBALT_SENSOR_POSES,
            COBALT_SENSOR_MAX_RANGE,
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
