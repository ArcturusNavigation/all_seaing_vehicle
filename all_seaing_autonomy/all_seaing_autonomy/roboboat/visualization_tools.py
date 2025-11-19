from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class VisualizationTools:

    @staticmethod
    def plot_line(x, y, id = 0, color = (1., 0., 0.), frame = "/base_link"):
        """
        Publishes the points (x, y) to publisher
        so they can be visualized in rviz as
        connected line segments.
        Args:
            x, y: The x and y values. These arrays
            must be of the same length.
            publisher: the publisher to publish to. The
            publisher must be of type Marker from the
            visualization_msgs.msg class.
            color: the RGB color of the plot.
            frame: the transformation frame to plot in.
        """
        # Construct a line
        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.header.frame_id = frame
        line_strip.id = id

        # Set the size and color
        line_strip.scale.x = 0.1
        line_strip.scale.y = 0.1
        line_strip.color.a = 1.
        line_strip.color.r = color[0]
        line_strip.color.g = color[1]
        line_strip.color.b = color[2]

        # Fill the line with the desired values
        for xi, yi in zip(x, y):
            p = Point()
            p.x = xi
            p.y = yi
            line_strip.points.append(p)

        # Publish the line
        return line_strip
    
    @staticmethod
    def visualize_segment(pt_left, pt_right, id = 0, color = (0.0, 0.0, 1.0), frame = '/base_link'):
        x_left, y_left = pt_left
        x_right, y_right = pt_right
        return VisualizationTools.plot_line([x_left,x_right],[y_left, y_right], id, color, frame)
    
    @staticmethod
    def visualize_line_params(line_params, id = 0, color = (0.0, 0.0, 1.0), frame = '/base_link'):
        (a, b, c), _ = line_params
        if(abs(b) > 0.001):
            return VisualizationTools.plot_line([-5.0,5.0],[(-a*(-5.0)-c)/b, (-a*(5.0)-c)/b], id, color, frame)
        else:
            return VisualizationTools.plot_line([-c/a,-c/a],[-5.0, 5.0], id, color, frame)
    
    @staticmethod
    def visualize_line(line_ctr, line_normal, id = 0, color = (0.0, 0.0, 1.0), frame = '/base_link'):
        x, y = line_ctr
        a, b = line_normal
        a_dir, b_dir = -b, a
        return VisualizationTools.plot_line([x-a_dir*5.0,x+a_dir*5.0],[y-b_dir*5.0,y+b_dir*5.0], id, color, frame)