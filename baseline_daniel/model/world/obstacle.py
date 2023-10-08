from geometry.polygon import Polygon
from geometry.point import Point
import math


class Obstacle:

    def __init__(self, polygon, pose):

        self.pose = pose
        self.polygon = polygon

        self.vx = 0
        self.vy = 0

        # Polygon translated and rotated according to the pose
        self.global_geometry = self.polygon.get_transformation_to_pose(self.pose)

    def set_velocity_vector(self, vx, vy):
        """
        A velocity vector represents the rate of change of the position of an object.
        The magnitude of a velocity vector gives the speed of an object while
        the vector direction gives its direction. This method sets the velocity vector
        starting from its vx and vy components.
        """

        self.vx = vx
        self.vy = vy

    def set_velocity_vector_from_magnitude_and_direction(self, ro, theta):
        """
        A velocity vector represents the rate of change of the position of an object.
        The magnitude of a velocity vector gives the speed of an object while
        the vector direction gives its direction. This method sets the velocity vector
        starting from its magnitude and orientation.
        """

        self.vx = ro * math.cos(theta)
        self.vy = ro * math.sin(theta)

    def step_motion(self, dt):

        # Step according to the current velocity
        self.pose.x += self.vx * dt
        self.pose.y += self.vy * dt

        # Update the global geometry
        self.global_geometry = self.polygon.get_transformation_to_pose(self.pose)


class RectangularObstacle(Obstacle):

    def __init__(self, width, height, pose):

        half_width_x = width * 0.5
        half_width_y = height * 0.5

        vertices = [
            Point(half_width_x, half_width_y),
            Point(half_width_x, -half_width_y),
            Point(-half_width_x, -half_width_y),
            Point(-half_width_x, half_width_y),
        ]
        geometry = Polygon(vertices)

        super().__init__(geometry, pose)
