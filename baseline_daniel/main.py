from geometry.polygon import Polygon
from model.robot.robot import Robot
from presenter.presenter import Presenter
from model.world.world import World
from view.backends.kivy.kivy_viewer import KivyView
from view.backends.gtk.gtk_viewer import GTKView

REFRESH_RATE = 20.0  # hertz

if __name__ == "__main__":

    # Initialize your robot and obstacles
    robot = Robot(
        Polygon([(0, 0), (20, 0), (20, 20), (0, 20)]),
                  (0, 0, 0))
    obstacles = []  # Add your obstacles here

    # Timing control
    period = 1.0 / REFRESH_RATE  # seconds

    # Create the world model
    model = World(period)

    # Add robot and obstacles to the world
    model.add_robot(robot)
    model.add_obstacle(obstacles)

    # Create instance of View
    view = KivyView()
    # view = GTKView()

    # Instantiate and initialize the presenter and run the simulation
    presenter = Presenter(model, view)
    presenter.start_app()
