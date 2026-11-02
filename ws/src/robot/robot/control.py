import rclpy
from robot.steady_node import SteadyNode
from msgs.msg import Point2D, WheelSpeeds


class Control(SteadyNode):

    def __init__(self):
        super().__init__('control')

        self.create_subscription(Point2D, "/robot/line", self.line_callback, 1)
        self.create_subscription(Point2D, "/robot/corner", self.corner_callback, 1)

        self.pub_wheels = self.create_publisher(WheelSpeeds, "/robot/wheels", 1)

        #default values :

    def line_callback(self, line_pos: Point2D):
        self.line_pos = line_pos

    def corner_callback(self, corner_pos: Point2D):
        self.corner_pos = corner_pos

    def apply_pid(self, target):
        pass

    def turn_90_degrees(self, left: bool):
        pass

    def send_wheel_speeds(self):
        pass


def main():
    rclpy.init()
    node = Control()
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
