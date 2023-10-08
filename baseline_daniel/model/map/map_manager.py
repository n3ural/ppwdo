from math import pi, sin, cos
from random import random, randrange
import pickle

from model.world.obstacle import RectangularObstacle

from geometry.point import Point
from geometry.polygon import Polygon

from geometry.utils import convex_polygon_intersect_test


# Environment parameters

# Max and minimum dimensions of obstacles
OBS_MIN_DIM = 0.1  # meters
OBS_MAX_DIM = 2.5  # meters
OBS_MAX_COMBINED_DIM = 2.6  # meters

# Number of obstacles
OBS_MIN_COUNT = 10
OBS_MAX_COUNT = 50

# Distance of each obstacle from the robot
OBS_MIN_DIST = 0.4  # meters
OBS_MAX_DIST = 6.0  # meters

# Goal distance from the robot
GOAL_MIN_DIST = 2.0  # meters
GOAL_MAX_DIST = 4.0  # meters

# Clearance around the goal
MIN_GOAL_CLEARANCE = 0.2  # meters


class MapManager:

    def __init__(self):

        self.current_obstacles = []
        self.current_goal = None

        # Obstacle params
        self.obs_min_dim = OBS_MIN_DIM
        self.obs_max_dim = OBS_MAX_DIM
        self.obs_max_combined_dim = OBS_MAX_COMBINED_DIM
        self.obs_min_count = OBS_MIN_COUNT
        self.obs_max_count = OBS_MAX_COUNT
        self.obs_min_dist = OBS_MIN_DIST
        self.obs_max_dist = OBS_MAX_DIST

        # Goal params
        self.goal_min_dist = GOAL_MIN_DIST
        self.goal_max_dist = GOAL_MAX_DIST

    def random_rectangular_obstacle(self):
        """
        Generate a random rectangular obstacle to be put in the map
        """

        obs_dim_range = self.obs_max_dim - self.obs_min_dim
        obs_dist_range = self.obs_max_dist - self.obs_min_dist

        # Generate dimensions
        width = self.obs_min_dim + (random() * obs_dim_range)
        height = self.obs_min_dim + (random() * obs_dim_range)
        while width + height > self.obs_max_combined_dim:
            height = self.obs_min_dim + (random() * obs_dim_range)

        # Generate position
        dist = self.obs_min_dist + (random() * obs_dist_range)
        phi = -pi + (random() * 2 * pi)
        x = dist * sin(phi)
        y = dist * cos(phi)

        # Generate orientation
        theta = -pi + (random() * 2 * pi)

        # Build a rectangular obstacle
        return RectangularObstacle(width, height, (x, y, theta))

    def random_polygonal_obstacle(self, num_vertex):
        """
        Generate a random polygonal obstacle with #num_vertex vertex to be put in the map
        """

        # TODO: add polygonal obstacle generation

        pass

    def random_map(self, world):

        # Generate the goal
        goal_dist_range = self.goal_max_dist - self.goal_min_dist
        dist = self.goal_min_dist + (random() * goal_dist_range)
        phi = -pi + (random() * 2 * pi)
        x = dist * sin(phi)
        y = dist * cos(phi)
        goal = Point(x, y)

        # Generate a proximity test geometry for the goal
        r = MIN_GOAL_CLEARANCE
        n = 6
        goal_test_geometry_points = []
        for i in range(n):
            goal_test_geometry_points.append(
                Point(x + r * cos(i * 2 * pi / n), y + r * sin(i * 2 * pi / n))
            )
        goal_test_geometry = Polygon(goal_test_geometry_points)

        # Generate the obstacles
        obstacles = []
        num_obstacles = randrange(self.obs_min_count, self.obs_max_count + 1)

        # Obstacles must not overlap any robot or the immediate surrounding of the goal
        test_geometries = [robot.global_geometry for robot in world.robots] + [goal_test_geometry]

        while len(obstacles) < num_obstacles:

            # Generate an obstacle
            obstacle = self.random_rectangular_obstacle()

            # Test if the obstacle overlaps the robots or the goal
            intersects = False
            for test_geometry in test_geometries:
                intersects |= convex_polygon_intersect_test(
                    test_geometry, obstacle.global_geometry
                )

            # TODO: if the obstacle intersects the goal or any of the robots, move it

            if not intersects:
                obstacles.append(obstacle)

        # Update the current obstacles and goal
        self.current_obstacles = obstacles
        self.current_goal = goal

        # apply the new obstacles and goal to the world
        self.apply_to_world(world)

    def save_map(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self.current_obstacles, file)
            pickle.dump(self.current_goal, file)

    def load_map(self, filename):
        with open(filename, "rb") as file:
            self.current_obstacles = pickle.load(file)
            self.current_goal = pickle.load(file)

    def apply_to_world(self, world):
        """
        Once the map has been generated we need to do two things:
        - add the obstacles to the world so that we can control them with the timing loop
        - notify the robots that they have a goal
        """

        # Add the current obstacles
        for obstacle in self.current_obstacles:
            world.add_obstacle(obstacle)

        # Program the robot supervisors
        for robot in world.robots:
            robot.supervisor.goal = self.current_goal
