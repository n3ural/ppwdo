from exceptions.collision_exception import CollisionException
from geometry.utils import check_nearness
from geometry.utils import convex_polygon_intersect_test


class World:

    def __init__(self, dt=0.05):

        # Initialize world time
        self.world_time = 0.0  # seconds
        self.dt = dt  # seconds

        # Initialize lists of world objects
        self.robots = []
        self.obstacles = []

        # Moving obstacles parameters
        self.obstacles_moving = False
        # TODO: add self.obstacles_bouncing_back = True

    def set_period(self, dt):
        self.dt = dt

    def step(self):
        """
        Step the simulation through one time interval
        """

        dt = self.dt

        # Step all the robots
        for robot in self.robots:
            robot.step_motion(dt)

        # Step all the obstacles
        if self.obstacles_moving:
            for obstacle in self.obstacles:
                obstacle.step_motion(dt)

        # step all the obstacles
        for obstacle in self.obstacles:
            # step obstacle
            obstacle.step_motion(dt)

        # Apply physics interactions
        self._apply_physics()

        # Increment world time
        self.world_time += dt

    def add_robot(self, robot):
        self.robots.append(robot)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def colliders(self):
        """
        Return all objects in the world that might collide with other objects in the
        world during simulation. The obstacles can overlap, so we don't need to consider them
        """

        return (
            self.robots
            # Obstacles move but we don't need to check for collisions between them
        )

    def solids(self):
        """
        Return all solids in the world
        """

        return self.robots + self.obstacles

    def _apply_physics(self):
        self._detect_collisions()
        # TODO: add _update_proximity_sensors()

    def _detect_collisions(self):
        """
        Test the world for existing collisions with solids.
        Raises a CollisionException if one occurs.
        """

        colliders = self.colliders()
        solids = self.solids()

        for collider in colliders:

            polygon1 = collider.global_geometry  # robot

            for solid in solids:

                # don't test an object against itself
                if solid is not collider:
                    polygon2 = solid.global_geometry  # obstacle

                    # Don't bother testing objects that are not near each other
                    if check_nearness(polygon1, polygon2):

                        if convex_polygon_intersect_test(polygon1, polygon2):
                            raise CollisionException()
