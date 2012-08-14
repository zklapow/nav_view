import roslib;roslib.load_manifest('nav_view')
import rospy

from QtGui import QWidget

class NavView(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

