import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

# Filter all the classes that are subclasses of Link
import inspect


class UnsupportedLinkElementException(Exception):
    pass


class Link:

    def __init__(self, geometry_type):
        self.geometry_type = geometry_type

    def __str__(self):
        return self.geometry_type


class BoxLink(Link):

    def __init__(self, width, height, depth):
        super().__init__('box')
        self.width = width
        self.height = height
        self.depth = depth

    def to_polygon(self):
        return Polygon([
            Point(0, 0),
            Point(0, 0 + self.width),
            Point(0 + self.width, 0 + self.width),
            Point(0 + self.width, 0)
        ])


class SphereLink(Link):

    def __init__(self, radius):
        super().__init__('sphere')
        self.radius = radius


class CylinderLink(Link):

    def __init__(self, radius, height):
        super().__init__('cylinder')
        self.radius = radius
        self.height = height


class URDFParser:

    def __init__(self):
        # Get all classes that inherit from Base
        all_classes = inspect.getmembers(
            inspect.getmodule(Base), inspect.isclass

        self.visual_regex = './/visual/geoemtry/'

    def parse_urdf(self, urdf_file):
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        links = {}
        for link in root.findall('.//link'):
            link_name = link.get('name')
            visuals = link.findall(self.visual_regex)


def parse_urdf(urdf_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    links = {}
    for link in root.findall(".//link"):
        link_name = link.get("name")

        visuals = link.findall(".//visual/geometry/box") + \
                  link.findall(".//visual/geometry/cylinder") + \
                  link.findall(".//visual/geometry/spgere")

        for visual in visuals:
            geometry_type = visual.tag
            if geometry_type == "box":
                dimensions = visual.attrib['size']
                links[link_name] = {"type": "box", "dimensions": dimensions}
            elif geometry_type == "cylinder":
                radius = visual.attrib['radius']
                length = visual.attrib['length']
                links[link_name] = {"type": "cylinder", "radius": radius, "length": length}
            elif geometry_type == "sphere":
                radius = visual.attrib['radius']
                links[link_name] = {"type": "sphere", "radius": radius}
            break  # Only consider the first visual element

    return links


def main():
    urdf_file = "robot.urdf"

    links = parse_urdf(urdf_file)

    polygons = []
    for link_name, mesh_filename in links.items():
        print(link_name)
        print(mesh_filename)

        # Assuming the mesh defines the vertices of the link's shape
        # You might need to adjust this depending on your URDF structure
        vertices = []  # List of (x, y) vertex coordinates
        # Parse the mesh file to extract vertices
        # ...

        # Create a Shapely Polygon from the vertices
        polygon = Polygon(vertices)
        polygons.append(polygon)

    # Combine all polygons to create a union
    combined_polygon = polygons[0].union(polygons[1:])

    # Extract the coordinates for plotting
    x, y = combined_polygon.exterior.xy

    # Plot the top view polygon
    plt.figure()
    plt.plot(x, y)
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Top View of Robot Polygon")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid()
    plt.show()


if __name__ == "__main__":
    main()
