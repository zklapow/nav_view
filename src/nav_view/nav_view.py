import roslib;roslib.load_manifest('nav_view')
import rospy

from nav_msgs.msg import OccupancyGrid

from QtCore import pyqtSignal
from QtGui import QWidget, QPixmap, QVBoxLayout, QLabel, QImage
from PIL import Image
from PIL.ImageQt import ImageQt

class NavView(QLabel):
    sig_up = pyqtSignal()
    def __init__(self):
        super(QWidget, self).__init__()
        self.setGeometry(300, 300, 250, 150)        

        self.sig_up.connect(self.pix_update)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb) 

    def map_cb(self, msg):
        self.resolution = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height

        data = []

        for c in msg.data:
            if c == 100:
                data.append((0, 0, 0))
            elif c == 0:
                data.append((255, 255, 255))
            else:
                data.append((127, 127, 127))

        im = Image.new('RGB', (self.w, self.h))
        im.putdata(data)
        self.image = im

        self.sig_up.emit()

    def pix_update(self):
        self.pix = ImageQt(self.image)
        self.setPixmap(QPixmap.fromImage(self.pix))
        print(self.pix.width())
        print(self.pix.height())
        self.show()
        
