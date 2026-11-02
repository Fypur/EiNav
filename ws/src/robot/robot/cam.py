import rclpy
from robot.steady_node import SteadyNode
from msgs.msg import Point2D


class Cam(SteadyNode):

    def __init__(self):
        super().__init__('cam')

        self.create_timer(0.1, self.process_camera)

        self.pub_line = self.create_publisher(Point2D, "/robot/line", 1)
        self.pub_corner = self.create_publisher(Point2D, "/robot/corner", 1)

    def process_camera(self):
        img = self.get_camera_feed()
        img = self.pretreat(img)

        line = self.detect_line(img)
        corner = self.detect_corner(img)

        self.pub_line.publish(Point2D(x=line[0], y=line[1]))
        self.pub_corner.publish(Point2D(x=corner[0], y=corner[1]))

# ------------------------------- TODO --------------------------------

    def get_camera_feed(self):
        """get the camera feed from the hardware"""
        return None

    def pretreat(self, img):
        """Grayscales the image and pretreats it"""
        return None

    def detect_corner(self, img):
        return (0., 0.)

    def detect_line(self, img):
        return (0., 0.)


# -----------------------------------------------------------------------


def main():
    rclpy.init()
    node = Cam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
