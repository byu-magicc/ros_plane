import os
import rospy
import rospkg
import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QListWidget, QLineEdit, QFileDialog
from python_qt_binding.QtCore import QRegExp
from python_qt_binding.QtGui import QRegExpValidator
import json
import os
from parse import parse,compile
import csv
from rosplane_msgs.msg import Waypoint



class WaypointPlanner(Plugin):

    def __init__(self, context):
        super(WaypointPlanner, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('WaypointPlanner')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('waypoint_planning'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Waypoint Planner')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        # Buttons
        self.clearWaypointsButton = self._widget.findChild(QPushButton,"clearWaypointsButton")
        self.clearWaypointsButton.clicked.connect(self.handleClearWaypoints)
        self.loadFileButton = self._widget.findChild(QPushButton,"loadFileButton")
        self.loadFileButton.clicked.connect(self.handleLoadFile)
        self.saveFileButton = self._widget.findChild(QPushButton,"saveFileButton")
        self.saveFileButton.clicked.connect(self.handleSaveFile)
        self.addWaypointButton = self._widget.findChild(QPushButton,"addWaypointButton")
        self.addWaypointButton.clicked.connect(self.handleAddWaypoint)
        self.deleteWaypointButton = self._widget.findChild(QPushButton,"deleteWaypointButton")
        self.deleteWaypointButton.clicked.connect(self.handleDeleteWaypoint)
        self.sendWaypointsButton = self._widget.findChild(QPushButton,"sendWaypointsButton")
        self.sendWaypointsButton.clicked.connect(self.handleSendWaypoints)
        #Text Edits
        self.locationLineEdit = self._widget.findChild(QLineEdit,"locationLineEdit")
        self.locationLineEdit.setText("50,50,-50")
        regexp = QRegExp("\d+,\d+,-\d+")
        validator = QRegExpValidator(regexp)
        self.locationLineEdit.setValidator(validator)
        regexp1 = QRegExp("\d+")
        validator1 = QRegExpValidator(regexp1)
        self.locationLineEdit.textChanged.connect(self.locationChanged)
        self.orientationLineEdit = self._widget.findChild(QLineEdit,"orientationLineEdit")
        self.orientationLineEdit.setText("45")
        self.orientationLineEdit.setValidator(validator1)
        self.orientationLineEdit.textChanged.connect(self.orientationChanged)
        self.velocityLineEdit = self._widget.findChild(QLineEdit,"velocityLineEdit")
        self.velocityLineEdit.setText("16")
        self.velocityLineEdit.setValidator(validator1)
        self.velocityLineEdit.textChanged.connect(self.velocityChanged)
        #Lists
        self.newWaypointList = self._widget.findChild(QListWidget,"newWaypointList")
        self.currentWaypointList = self._widget.findChild(QListWidget,"currentWaypointList")
        self.waypoints = []
        self.currentWaypoints = []
        self.currentWaypoint = None
        self.newWaypointList.itemClicked.connect(self.newWaypointSelected)
        self.fileDialog = QFileDialog()
        self.fileDialog.setFileMode(QFileDialog.AnyFile)
        #Publisher/Subscriber
        self.publisher = rospy.Publisher('waypoint_path',Waypoint,queue_size=20)
        self.subscriber = rospy.Subscriber('waypoint_path',Waypoint,self.publishedWaypointCallback)

    def updateWaypoint(self):
        self.newWaypointList.item(self.waypoints.index(self.currentWaypoint)).setText(self.waypointToString(self.currentWaypoint))

    def locationChanged(self,text):
        try:
            (x,y,z) = parse('{},{},{}',text)
            if self.currentWaypoint is not None and x is not None and y is not None and z is not None and z is not "-":
                self.currentWaypoint.w[0] = float(x)
                self.currentWaypoint.w[1] = float(y)
                self.currentWaypoint.w[2] = float(z)
                self.updateWaypoint()
        except:
            print("Invalid location")

    def orientationChanged(self,text):
        if self.currentWaypoint is not None and text is not None and text is not "":
            try:
                temp = float(text)*2*math.pi/360 #convert to radians
                self.currentWaypoint.chi_d = temp
                self.updateWaypoint()
            except:
                print("invalid orientation string")


    def velocityChanged(self,text):
        if self.currentWaypoint is not None and text is not "":
            self.currentWaypoint.Va_d = float(text)
            self.updateWaypoint()

    def waypointToString(self,waypoint):
        return "Waypoint X:{} Y:{} Z:{} @{}degrees, {}kph".format(waypoint.w[0],waypoint.w[1],waypoint.w[2],waypoint.chi_d*180/math.pi,waypoint.Va_d)

    def createWaypoint(self,waypoint):
        self.waypoints.append(waypoint)
        self.newWaypointList.addItem(self.waypointToString(waypoint))

    def newWaypointSelected(self,item):
        self.currentWaypoint = self.waypoints[self.newWaypointList.currentRow()]

    def handleClearWaypoints(self):
        roswaypoint = Waypoint()
        roswaypoint.w[0] = 0.0
        roswaypoint.w[1] = 0.0
        roswaypoint.w[2] = 0.0
        roswaypoint.chi_d = 0.0
        roswaypoint.chi_valid = False
        roswaypoint.Va_d = 0.0
        roswaypoint.clear_wp_list = True
        roswaypoint.set_current = True
        self.publisher.publish(roswaypoint)


    def handleLoadFile(self):
        file_name = self.fileDialog.getOpenFileName(self._widget,'Open File', '/home/',"Waypoint Files (*.wp)")
        if not file_name[0]:
            return
        print("OpenFileName:{}".format(file_name[0]))
        f = open(file_name[0],'r')
        csvReader = csv.reader(f)
        self.waypoints = []
        for row in csvReader:
            print(row)
            waypoint = Waypoint()
            waypoint.w[0] = float(row[0])
            waypoint.w[1] = float(row[1])
            waypoint.w[2] = float(row[2])
            temp = float(row[3])*2*math.pi/360
            waypoint.chi_d = temp
            waypoint.Va_d = float(row[4])
            self.waypoints.append(waypoint)
        f.close()
        self.newWaypointList.clear()
        for waypoint in self.waypoints:
            self.newWaypointList.addItem(self.waypointToString(waypoint))


    def handleSaveFile(self):
        file_name = self.fileDialog.getSaveFileName(self._widget,'Save File', '/home/',"Waypoint Files (*.wp)")
        if not file_name[0]:
            return
        print("SaveFileName:{}".format(file_name[0]))
        try:
            f = open(file_name[0],'w')
        except:
            print("Could not save file. Permissions may have been denied")
            return
        csvWriter = csv.writer(f)
        for waypoint in self.waypoints:
            csvWriter.writerow([waypoint.w[0],waypoint.w[1],waypoint.w[2],waypoint.chi_d*180/math.pi,waypoint.Va_d])
        f.close()


    def handleAddWaypoint(self):
        text = self.locationLineEdit.text()
        (x,y,z) = parse('{},{},{}',text)
        orientation = self.orientationLineEdit.text()
        velocity = self.velocityLineEdit.text()
        waypoint = Waypoint()
        waypoint.w[0] = float(x)
        waypoint.w[1] = float(y)
        waypoint.w[2] = float(z)
        temp = float(orientation)*2*math.pi/360
        waypoint.chi_d = temp
        waypoint.chi_valid = False
        waypoint.Va_d = float(velocity)
        waypoint.clear_wp_list = False
        waypoint.set_current = False
        self.createWaypoint(waypoint)
        self.currentWaypoint = waypoint


    def handleDeleteWaypoint(self):
        if not self.waypoints or self.currentWaypoint not in self.waypoints:
            print("Can't remove, waypoint not selected or no more waypoints")
            return
        self.waypoints.remove(self.currentWaypoint)
        self.newWaypointList.clear()
        for waypoint in self.waypoints:
            self.newWaypointList.addItem(self.waypointToString(waypoint))



    def publishedWaypointCallback(self,waypoint):
        if waypoint.clear_wp_list == True:
            self.currentWaypoints = []
            self.currentWaypointList.clear()
        else:
            self.currentWaypointList.addItem(self.waypointToString(waypoint))


    def handleSendWaypoints(self):
        first = True
        for waypoint in self.waypoints:
            if first:
                waypoint.set_current = True
                first = False
            self.publisher.publish(waypoint)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog