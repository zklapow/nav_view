import roslib;roslib.load_manifest('nav_view')
import rospy

from nav_view import NavView
from qt_gui.plugin import Plugin

class RobotMonitorPlugin(Plugin):
    def __init__(self, context):
        super(RobotMonitorPlugin, self).__init__(context)
        context.add_widget(NavView())

        self.setObjectName('Naviation View')

