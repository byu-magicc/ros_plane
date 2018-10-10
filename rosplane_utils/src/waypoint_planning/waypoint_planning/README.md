ROSplane Plugins
=================

This ROS package contains basic plugins for controlling a uav, setting waypoints etc..

The waypoint planning node allows dynamic changes (without rewriting
code or rerunning nodes) of the waypoint path.

It's functionality is best observed by opening Gazebo and launching the
rosplane groundstation.

As shown in the GUI, the points listed on the right represent the
current waypoints the plane is/will attempt to reach. Clicking the clear
list button clears the plane's current waypoints completely and returns the plane to
loiter about the origin.

On the left are waypoints to be sent to the plane. These can be added
and edited using the text boxes below, but will not be executed by the
plane until sent to the waypoint list.

Finally, waypoint lists from the left column can be saved and loaded
for later use using the corresponding buttons. 