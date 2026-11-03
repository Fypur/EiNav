def process_camera(self):
    img = self.pretreat(self.get_camera_feed())

    line = self.detect_line(img)
    corner = self.detect_corner(img)

    self.pub_line.publish(line)
    self.pub_corner.publish(corner)


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
