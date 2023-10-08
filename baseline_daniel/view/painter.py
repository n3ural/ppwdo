from color_palette import ColorPalette
from math import pi


class Painter:
    """
    The painter class is agnostic and can work with different graphics libraries.
    We have an abstract interface for drawing context operations; we must add
    specific adapters for each graphics library (e.g. GTK, Kivy, ...)
    """

    def __init__(self, pixels_per_meter):
        self.pixels_per_meter = pixels_per_meter

    def draw_frame(self, frame, drawing_context):

        # Draw the background in white
        self.set_color(drawing_context, "white", 1.0)
        drawing_context.paint()

        draw_list = frame.draw_list
        for component in draw_list:
            if component["type"] == "circle":
                self.draw_circle(
                    drawing_context,
                    component["pos"],
                    component["radius"],
                    component["color"],
                    component["alpha"],
                )

            elif component["type"] == "polygons":
                self.draw_polygons(
                    drawing_context,
                    component["polygons"],
                    component["color"],
                    component["alpha"],
                )

            elif component["type"] == "lines":
                self.draw_lines(
                    drawing_context,
                    component["lines"],
                    component["line_width"],
                    component["color"],
                    component["alpha"],
                )

    def draw_circle(self, context, pos, radius, color, alpha):
        self.set_color(context, color, alpha)
        context.arc(pos[0], pos[1], radius, 0, 2.0 * pi)
        context.fill()

    def draw_polygons(self, context, polygons, color, alpha):
        self.set_color(context, color, alpha)
        for polygon in polygons:
            context.new_path()
            context.move_to(*polygon[0])
            for point in polygon[1:]:
                context.line_to(*point)
            context.fill()

    def draw_lines(self, context, lines, line_width, color, alpha):
        self.set_color(context, color, alpha)
        context.set_line_width(line_width)
        for line in lines:
            context.new_path()
            context.move_to(*line[0])
            for point in line[1:]:
                context.line_to(*point)
            context.stroke()

    @classmethod
    def set_color(cls, context, color_string, alpha):
        ColorPalette.dab(context, color_string, alpha)
