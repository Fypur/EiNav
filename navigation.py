import rclpy
from robot.steady_node import SteadyNode


class Navigation(SteadyNode):

    def __init__(self):
        super().__init__('navigation')


def main():
    rclpy.init()
    node = Navigation()
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
