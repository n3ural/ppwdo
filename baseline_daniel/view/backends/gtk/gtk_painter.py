from view.color_palette import color_table
from math import pi


class GTKPainter:

    def __init__(self, widget, context, pixels_per_meter=100):

        """
        cairo is a 2D graphics library that GTK uses for rendering graphics, and the cairo.Context object
        represents the drawing context for the Gtk.DrawingArea or other graphics components. In this case
        we need to use the context to draw something.
        """

        self.widget = widget
        self.context = context

        self.pixels_per_meter = pixels_per_meter

    def draw_frame(self, frame):

        # Get the dimensions of the drawing area
        width_pixels = self.widget.get_allocated_width()
        height_pixels = self.widget.get_allocated_height()

        # Transform the view to metric coordinates
        self.context.translate(
            width_pixels / 2.0, height_pixels / 2.0
        )  # Move origin to center of window
        self.context.scale(
            self.pixels_per_meter, -self.pixels_per_meter
        )  # Pull view to edges of window ( also flips y-axis )

        # Set the color
        self.set_color("white", 1.0)

        # Draw all the elements in the frame
        for component in frame.draw_list:
            if component["type"] == "circle":
                self.draw_circle(
                    component["pos"],
                    component["radius"],
                    component["color"],
                    component["alpha"],
                )

            elif component["type"] == "polygon":
                self.draw_polygon(
                    component["polygon"],
                    component["color"],
                    component["alpha"],
                )

            elif component["type"] == "line":
                self.draw_line(
                    component["line"],
                    component["line_width"],
                    component["color"],
                    component["alpha"],
                )

    def set_color(self, color_string, alpha):
        color_vals = [c / 255.0 for c in color_table[color_string]]
        if alpha:
            self.context.set_cource_rgba(color_vals[0], color_vals[1], color_vals[2], alpha)
        else:
            self.context.set_source_rgb(color_vals[0], color_vals[1], color_vals[2])

    def draw_circle(self, pos, radius, color_string, alpha):
        self.set_color(color_string, alpha)
        self.context.arc(pos[0], pos[1], radius, 0, 2.0 * pi)
        self.context.fill()

    def draw_line(self, line, line_width, color_string, alpha):
        self.set_color(color_string, alpha)
        self.context.set_line_width(line_width)
        self.context.new_path()
        self.context.move_to(line[0])
        for point in line[1:]:
            self.context.line_to(point)
        self.context.stroke()

    def draw_polygon(self, polygon, color_string, alpha):
        self.set_color(color_string, alpha)
        self.context.new_path()
        self.context.move_to(polygon[0])
        for point in polygon[1:]:
            self.context.line_to(point)
        self.context.stroke()
