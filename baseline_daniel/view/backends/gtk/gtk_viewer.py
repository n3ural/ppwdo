import gi

from view.backends.gtk.gtk_painter import GTKPainter

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk

from view.viewer import Viewer
from view.frame import Frame


class GTKView(Viewer):

    def __init__(self):

        super(GTKView, self).__init__()

        # Initialize frame
        self.current_frame = Frame()

        # Initialize the window
        self.window = Gtk.Window()
        self.window.set_title("Robot Simulator")
        self.window.set_resizable(False)
        self.window.connect("delete_event", self.on_delete)

        # Initialize the drawing area
        self.drawing_area = Gtk.DrawingArea()
        self.drawing_area.set_size_request(
            self.view_width_pixels, self.view_height_pixels
        )
        # self.drawing_area.connect("draw", self.on_expose)

        # Build the play button
        self.button_play = Gtk.Button(label="Play")
        self.button_play.connect("clicked", self.play_callback)

        # Build the stop button
        self.button_stop = Gtk.Button(label="Stop")
        self.button_stop.connect("clicked", self.stop_callback)

        # Build the step button
        self.button_step = Gtk.Button(label="Step")
        self.button_step.connect("clicked", self.step_callback)

        # Pack the simulation control buttons
        sim_controls_box = Gtk.HBox(spacing=5)
        sim_controls_box.pack_start(self.button_play, False, False, 0)
        sim_controls_box.pack_start(self.button_stop, False, False, 0)
        sim_controls_box.pack_start(self.button_step, False, False, 0)

        # Create the simulation control box
        sim_controls_alignment = Gtk.Alignment(
            xalign=0.5, yalign=0.5, xscale=0, yscale=0
        )
        sim_controls_alignment.add(sim_controls_box)

        # Create the alert box
        self.alert_box = Gtk.Label()

        layout_box = Gtk.VBox()
        layout_box.pack_start(self.drawing_area, False, False, 0)
        layout_box.pack_start(self.alert_box, False, False, 5)
        layout_box.pack_start(sim_controls_alignment, False, False, 5)

        self.window.add(layout_box)

        self.window.show_all()

        # Prepare the painter
        self.painter = GTKPainter(self.drawing_area, )

    def on_delete(self, widget, event):
        Gtk.main_quit()
        return False

    def show_all(self):
        Gtk.main()


if __name__ == '__main__':
    GTKView().show_all()
