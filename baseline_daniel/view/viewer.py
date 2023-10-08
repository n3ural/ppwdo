from view.signal import Signal
from view.frame import Frame

import mss

'''
Cross-platform dependence-free library that supports multiple monitors 
'''
sct = mss.mss()
MONITOR_WIDTH = sct.monitors[0]['width']
MONITOR_HEIGHT = sct.monitors[0]['height']

DEFAULT_VIEW_PIX_W = round(MONITOR_WIDTH * 0.5)  # pixels
DEFAULT_VIEW_PIX_H = round(MONITOR_HEIGHT * 0.7)  # pixels
DEFAULT_ZOOM = 100  # pixels per meter


class Viewer:
    """
    View interface that abstracts all the implementations (GTK, Kivy, ...).
    Ideally, the view interface should encapsulate the state of the view:
    it should expose a boolean for every checkbox and button, a string for
    every input field and so on. The presenter can then get information
    from the view with getters, without having knowledge about the
    components (checkboxes, buttons and so on) that are library specific

    +------------------------------+----------------+
    |                              |                |
    |                              | map control    |
    |            drawing           |                |
    |             area             | speed control  |
    |                              |                |
    |                              | ...            |
    +------------------------------+                |
    |      play | stop | step      |                |
    +------------------------------+----------------+

    This is the outline of the view. Each implementation should have
    the same layout, no matter how. There must be a signal and a state
    variable for each component
    """

    def __init__(self):

        # Initialize camera parameters
        self.view_width_pixels = DEFAULT_VIEW_PIX_W
        self.view_height_pixels = DEFAULT_VIEW_PIX_H
        self.pixels_per_meter = DEFAULT_ZOOM

        # Create a signal for each button
        self.play_button_signal = Signal()
        self.stop_button_signal = Signal()
        self.step_button_signal = Signal()

        # Prepare the frame
        self.frame = Frame()

        # Prepare the painter
        self.painter = None

    def play_callback(self, instance):
        self.play_button_signal.emit()

    def stop_callback(self, instance):
        self.stop_button_signal.emit()

    def step_callback(self, instance):
        self.step_button_signal.emit()

    def show_all(self):
        raise NotImplemented('Not implemented for generic Viewer interface')

    def update(self):
        raise NotImplemented('Not implemented for generic Viewer interface')
