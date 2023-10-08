from exceptions.collision_exception import CollisionException
from exceptions.goal_reached_exception import GoalReachedException
from view.frame import Frame
import threading
import time


# REFRESH_RATE = 20.0  # hertz
REFRESH_RATE = 1  # hertz


class Presenter:

    def __init__(self, world, viewer):

        self.viewer = viewer
        self.world = world

        # Bind the view
        self.viewer.play_button_signal.connect(self.play)
        self.viewer.stop_button_signal.connect(self.stop)
        self.viewer.step_button_signal.connect(self.step)

        # Create and start the background thread
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.started = False

        self.flag = threading.Event()  # The flag used to suspend the thread
        self.flag.set()  # set to True

        self.running = threading.Event()  # ID used to stop the thread
        self.running.set()  # set to True

    def _run(self):

        while self.running.is_set():
            self.flag.wait()  # Returns immediately when True, blocks when False until the internal flag is True and returns

            # Actual function
            self._step_sim()

            time.sleep(1 / REFRESH_RATE)

    def play(self):
        print('Presenter -> start')
        if not self.started:
            self.started = True
            self.thread.start()
        else:
            # Resume
            self.flag.set()

    def stop(self):
        print('Presenter -> stop')
        self.flag.clear()  # Set to False to block the thread

    def step(self):
        print('Presenter -> step')
        # self.paused.clear()
        self._step_sim()

    def start_app(self):
        self.viewer.show_all()

    def _step_sim(self):

        # Try to increment the simulation
        try:

            # 1. Controller computes next step
            # 2. Robot translates step from velocity vector to coordinates
            # 3. Update robot
            # 4. Update obstacles based on their velocities
            # 5. Compute collisions

            print('Step world')
            # self.world.step()

        except CollisionException:
            raise CollisionException("Collision!")
        except GoalReachedException:
            raise GoalReachedException("Goal Reached!")

        # Draw the resulting world
        self._draw_world()

    def _draw_world(self):

        # self.viewer.drawing_area.frame.clear()
        self.viewer.frame.clear()

        # Add all the obstacles
        # self.viewer.drawing_area.frame.add_polygons(self.world.obstacles, color="dark red", alpha=0.4)
        self.viewer.frame.add_polygons(self.world.obstacles, color="dark red", alpha=0.4)

        # Add the robots
        #self.viewer.drawing_area.frame.add_polygons(self.world.robots, color="blue", alpha=0.5)
        self.viewer.frame.add_circles(
            [(robot.pose[0], robot.pose[1]) for robot in self.world.robots],
            [5 for robot in self.world.robots],
            color="blue",
            alpha=0.5
        )

        self.viewer.update()

