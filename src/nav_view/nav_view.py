import roslib;roslib.load_manifest('nav_view')
import rospy

import random

from nav_msgs.msg import OccupancyGrid, Path

from QtCore import pyqtSignal
from QtGui import QWidget, QPixmap, QVBoxLayout, QLabel, QImage, QGraphicsView, QGraphicsScene, QPainterPath, QPen, QColor

from PIL import Image
from PIL.ImageQt import ImageQt

class PathInfo(object):
    def __init__(self, name=None):
        self.color = None
        self.sub = None
        self.cb = None
        self.path = None
        self.item = None

        self.name = name

class NavView(QGraphicsView):
    sig_map = pyqtSignal()
    path_changed = pyqtSignal(str)
    def __init__(self, map_topic = '/map', paths = ['/move_base/TrajectoryPlannerROS/global_plan', '/move_base/SBPLLatticePlanner/plan'], polygons= []):
        super(QWidget, self).__init__()
        self.sig_map.connect(self._update)
        self.destroyed.connect(self.close)
        self._map = None
        self._map_item = None

        self._paths = {}
        self.path_changed.connect(self._update_path)

        self._colors = [(238, 34, 116), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]

        self._scene = QGraphicsScene()

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb) 

        for path in paths:
            self.add_path(path)

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
        self._map = im

        self.sig_map.emit()

    def add_path(self, name):
        path = PathInfo(name)
        def c(msg):
            pp = QPainterPath()
            start = msg.poses[0].pose.position
            pp.moveTo(start.x / self.resolution, start.y / self.resolution)
            for pose in msg.poses:
                pt = pose.pose.position
                pp.lineTo(pt.x / self.resolution, pt.y / self.resolution)

            path.path = pp
            self.path_changed.emit(name)

        path.color = random.choice(self._colors)
        self._colors.remove(path.color)

        path.cb = c
        path.sub = rospy.Subscriber(path.name, Path, path.cb)

        self._paths[name] = path

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
        self._mirror(self._map_item)

        self.centerOn(self._map_item)
        self.show()

    def _update_path(self, name):
        try:
            self._scene.removeItem(self._paths[name].item)
        except:
            pass

        self._paths[name].item = self._scene.addPath(self._paths[name].path, pen = QPen(QColor(*self._paths[name].color)))

        # Everything must be mirrored
        self._mirror(self._paths[name].item)

    def _mirror(self, item):
        item.scale(-1,1)
        item.translate(-self.w, 0)

