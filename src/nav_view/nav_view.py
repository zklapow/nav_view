import roslib;roslib.load_manifest('nav_view')
import rospy

from nav_msgs.msg import OccupancyGrid, Path

from QtCore import pyqtSignal
from QtGui import QWidget, QPixmap, QVBoxLayout, QLabel, QImage, QGraphicsView, QGraphicsScene, QPainterPath

from PIL import Image
from PIL.ImageQt import ImageQt

def mirror(item):
    item.scale(-1,1)
    item.translate(-item.boundingRect().width(), 0)

class NavView(QGraphicsView):
    sig_map = pyqtSignal()
    path_changed = pyqtSignal(str)
    def __init__(self):
        super(QWidget, self).__init__()
        self.sig_map.connect(self._update)
        self.destroyed.connect(self.close)
        self._map = None
        self._map_item = None

        self._path_cbs = {}
        self._path_subs = {}
        self._path_items = {}
        self._paths = {}
        self.path_changed.connect(self._update_path)

        self._scene = QGraphicsScene()

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb) 

        self.add_path('/move_base/SBPLLatticePlanner/plan')

        self.setScene(self._scene)

    def map_cb(self, msg):
        self.resolution = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height

        data = []

        # Get correct colors
        for c in msg.data:
            if c == 100:
                data.append((0, 0, 0))
            elif c == 0:
                data.append((255, 255, 255))
            else:
                data.append((127, 127, 127))

        im = Image.new('RGB', (self.w, self.h))
        im.putdata(data)
    
        #Occupancy grid is mirrored
        self._map = im

        self.sig_map.emit()

    def add_path(self, name):
        def c(msg):
            path = QPainterPath()
            start = msg.poses[0].pose.position
            path.moveTo(start.x / self.resolution, start.y / self.resolution)
            for pose in msg.poses:
                pt = pose.pose.position
                path.lineTo(pt.x / self.resolution, pt.y / self.resolution)

            self._paths[name] = path
            self.path_changed.emit(name)

        self._path_cbs[name] = c
        self._path_subs = rospy.Subscriber(name, Path, self._path_cbs[name])

    def close(self):
        if self.map_sub:
            self.map_sub.unregister()

    def _update(self):
        if self._map_item:
            self._scene.removeItem(self._map_item)

        self.pix = ImageQt(self._map)
        self.setSceneRect(0,0, self.w, self.h)
        self._map_item = self._scene.addPixmap(QPixmap.fromImage(self.pix))

        # Everything must be mirrored
        mirror(self._map_item)

        self.centerOn(self._map_item)
        self.show()

    def _update_path(self, name):
        if name in self._path_items.keys():
            self._scene.removeItem(self._path_items[name])

        self._path_items[name] = self._scene.addPath(self._paths[name])
        self._path_items[name].scale(-1, 1)
        self._path_items[name].translate(-self._map_item.boundingRect().width(), 0)

